#include "solver.h"

#include <opencv2/core/mat.hpp>

#include <spdlog/spdlog.h>

#include <opencv2/imgcodecs.hpp> // temp for cv::imwrite

#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/frame.h>
#include <stella_vslam/data/map_camera_helpers.h>
#include <stella_vslam/data/map_database.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/report/plot_html.h>
#include <stella_vslam/report/metrics.h>
#include <stella_vslam/config.h>
#include <stella_vslam/feature/orb_preanalysis.h>
#include <stella_vslam/module/frame_track_repositioning.h>
#include <stella_vslam/solve/fundamental_consistency.h>

#include "type.h"
#include "system.h"
#include "tracking_module.h"

using namespace stella_vslam;

namespace stella_vslam_bfx {

void solve::clear() {
    frame_to_camera.clear();
    world_points.clear();
    prematched_id_to_idx.clear();
    camera_lens.reset();
}

void frame_display_data::clear() {
    frame = -1;
    final_points = false;
    solve_success = false;
    focal_length = 0.0;
    camera_pose.setIdentity();
    world_points.clear();
    prematched_id_to_idx.clear();
}

solver::solver(const std::shared_ptr<config>& cfg,
               const std::string& vocab_file_path,
               std::function<bool(int, cv::Mat&, cv::Mat&, stella_vslam_bfx::prematched_points&)> get_frame)
: get_frame_(get_frame), cfg_(cfg), vocab_data_(nullptr), vocab_file_path_(vocab_file_path)
{
    // Make sure that static objects are cleared between solver instances
    metrics::clear();
    focal_length_estimator::clear();
    focal_length_estimator_three_view::clear();

    // build the slam system
    slam_ = std::make_shared<stella_vslam::system>(cfg, vocab_file_path);

    bool already_have_map(false); // see run_video_slam for the case where we already have a map
    slam_->startup(!already_have_map);
}

#if !defined(USE_DBOW2)
solver::solver(const std::shared_ptr<config>& cfg,
               std::ifstream & vocab_data,
               std::function<bool(int, cv::Mat&, cv::Mat&, stella_vslam_bfx::prematched_points&)> get_frame)
: get_frame_(get_frame), cfg_(cfg), vocab_data_(&vocab_data), vocab_file_path_()
{
    // Make sure that static objects are cleared between solver instances
    metrics::clear();
    focal_length_estimator::clear();
    focal_length_estimator_three_view::clear();

    slam_ = std::make_shared<stella_vslam::system>(cfg, vocab_data);

    bool already_have_map(false); // see run_video_slam for the case where we already have a map
    slam_->startup(!already_have_map);
}
#endif

solver::~solver() {
    // shutdown the slam process
    if (slam_)
        slam_->shutdown();
}

void solver::set_progress_callback(std::function<void(float)> set_progress) {
    set_progress_ = set_progress;
}

void solver::set_stage_description_callback(std::function<void(std::string)> set_stage_description) {
    set_stage_description_ = set_stage_description;
}

void solver::set_display_frame_callback(std::function<void(std::shared_ptr<frame_display_data>)> display_frame) {
    display_frame_ = display_frame;
}

void solver::set_cancel_callback(std::function<bool()> cancel) {
    cancel_ = cancel;
}

double overall_progress_percent(double stage_progress, double stage_start_progress, double stage_end_progress) {
    return 100.0 * (stage_start_progress + stage_progress * (stage_end_progress - stage_start_progress));
}

void print_map_summary(std::string const& name,
                       data::map_database const& map_db,
                       std::map<double, stage_and_frame> & timestamp_to_stage_and_frame, 
                       bool verbose)
{
    std::vector<std::shared_ptr<data::keyframe>> keys = map_db.get_all_keyframes();
    std::vector<std::shared_ptr<data::landmark>> local_landmarks = map_db.get_local_landmarks();
    std::vector<std::shared_ptr<data::landmark>> all_landmarks = map_db.get_all_landmarks();

    spdlog::info("==========================");
    spdlog::info("Map: {}", name);
    spdlog::info("  Keyframes: {}", keys.size());
    spdlog::info("  Landmarks: {}", all_landmarks.size());
    spdlog::info("  Local landmarks: {}", local_landmarks.size());
    if (verbose) {
        spdlog::info("");
        for (auto const& key : keys)
            spdlog::info("    Key id: {}, frame {}, valid landmarks: {}", key->id_, timestamp_to_stage_and_frame[key->timestamp_].frame, key->get_valid_landmarks().size());
    }
    spdlog::info("==========================");
}

bool solver::track_frame_range(int begin, int end, tracking_direction direction, solve* final_solve) {

    cv::Mat frame_image;
    cv::Mat mask;
    prematched_points extra_keypoints;
    if (final_solve)
        final_solve->clear();

    if (!get_frame_)
        return false; // report error!

    // Initialise metrics to an unsolved state
    metrics& track_metrics = *metrics::get_instance();
    track_metrics.calculated_focal_length_x_pixels = -1;
    track_metrics.solved_frame_count = 0;
    track_metrics.unsolved_frame_count = end - begin + 1;
    track_metrics.num_points = 0;

    // ==Timestamps==
    // Stella uses timestamps, rather than frame numbers internally
    // Some of the tracking logic depends on the timestamp value of frames
    //  e.g. Don't insert a keyframe within a second of a relocalisation (tracking_module::new_keyframe_is_needed())
    //  e.g. If tracking fails within 60.0 sec of initialization, reset the system (initializer::try_initialize_for_monocular) (though we bypass this one)
    // So when feeding frames to stella, we make sure the associated timestamp is always increasing, even if we are re-feeding frames which were previously sent with older timestamps, or tracking backwards
    std::map<double, stage_and_frame> timestamp_to_stage_and_frame;
    metrics::get_instance()->timestamp_to_stage_and_frame = &timestamp_to_stage_and_frame;
    double const timestamp_increment(1.0); // Seconds
    double next_timestamp(0);

    // To do, support the tracking direction
    if (direction == tracking_direction_backwards)
        return false;

    // Pre-analyse the video to adjust min_feature_size if it doesn't result in enough matches
    if (!use_feature_monitor()) {
        cv::Mat frame_image_1, frame_image_2;
        cv::Mat mask_1, mask_2;
        prematched_points extra_keypoints_1, extra_keypoints_2;
        int frame_1(begin), frame_2(begin+1);
        bool got_frame_1 = get_frame_(frame_1, frame_image_1, mask_1, extra_keypoints_1);
        bool got_frame_2 = get_frame_(frame_2, frame_image_2, mask_2, extra_keypoints_2);
        if (got_frame_1 && !frame_image_1.empty() && got_frame_2 && !frame_image_2.empty()) {
            stella_vslam_bfx::preanalysis(frame_image_1, mask_1, frame_image_2, mask_2, slam_.get());
        }
    }

    // Track forwards to create the map
    if (set_stage_description_)
        set_stage_description_("Analysing video");
    if (set_progress_)
        set_progress_(0);
    const auto tp_start = std::chrono::steady_clock::now();
    slam_->tracker_->map_selector_.set_system(slam_);
    slam_->enable_map_reinitialisation(true);
    for (int frame = begin; frame <= end; ++frame) {
        bool got_frame = get_frame_(frame, frame_image, mask, extra_keypoints);
        if (!got_frame || frame_image.empty())
            continue;

        double timestamp = next_timestamp;
        next_timestamp += timestamp_increment;
        timestamp_to_stage_and_frame[timestamp] = { 0, frame };
        metrics::get_instance()->current_frame_timestamp = timestamp;

        auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, &extra_keypoints);
        send_frame_data(frame, slam_->get_current_frame().camera_, camera_pose, false, true);

        double stage_progress = double(frame - begin + 1) / double(1 + end - begin);
        if (set_progress_)
            set_progress_(overall_progress_percent(stage_progress, 0.0, 0.4));
        if (cancel_ && cancel_())
            return false;
    }

    // Restore a better map if there is one
    if (auto better_map = slam_->tracker_->map_selector_.get_best_map(*slam_->map_db_)) {
        // recreate the slam system
        if (!vocab_file_path_.empty())
            slam_ = std::make_shared<stella_vslam::system>(cfg_, vocab_file_path_);
        else if (vocab_data_)
            slam_ = std::make_shared<stella_vslam::system>(cfg_, *vocab_data_);

        slam_->startup(false); // false sets the tracking state to Lost, rather than to Initializing
        bool load_was_ok = slam_->load_map_database_from_memory(*better_map);
        if (load_was_ok)
            spdlog::info("An abandoned map was restored");
    }
    const auto tp_after_forward_mapping = std::chrono::steady_clock::now();
    track_metrics.track_timings.forward_mapping = std::chrono::duration_cast<std::chrono::duration<double>>(tp_after_forward_mapping - tp_start).count();

    // Reposition tracking to the start of the map before tracking backwards
    int reposition_frame;
    bool track_repositioning_ok = reposition_tracking_to_first_map_keyframe(slam_, timestamp_to_stage_and_frame, reposition_frame);
    if (!track_repositioning_ok) {
        spdlog::error("No map to continue tracking from after first pass");
        track_metrics.create_frame_metrics();
        return false;
    }

    // Track backwards from the initialization point to finish the map
    if (set_stage_description_)
        set_stage_description_("Revisiting video");
    slam_->enable_map_reinitialisation(false); // Prevent the map from being abandoned
    int pass_2_start_frame(reposition_frame); // or -1
    metrics::get_instance()->pass_2_frames = 1 + pass_2_start_frame - begin;
    for (int frame = pass_2_start_frame; frame >= begin; --frame) {

        bool got_frame = get_frame_(frame, frame_image, mask, extra_keypoints);
        if (!got_frame || frame_image.empty())
            continue;

        double timestamp = next_timestamp;
        next_timestamp += timestamp_increment;
        timestamp_to_stage_and_frame[timestamp] = { 1, frame };
        metrics::get_instance()->current_frame_timestamp = timestamp;

        auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, &extra_keypoints);
        send_frame_data(frame, slam_->get_current_frame().camera_, camera_pose, false, true);

        double stage_progress = double(reposition_frame - frame) / double(reposition_frame - begin);
        if (set_progress_)
            set_progress_(overall_progress_percent(stage_progress, 0.4, 0.5));
        if (cancel_ && cancel_())
            return false;

        spdlog::info("Tracking backwards at {} of {} done", frame, begin);
    }
    metrics::get_instance()->pass_2_end_keyframes = slam_->map_db_->get_num_keyframes();
    const auto tp_after_backward_mapping = std::chrono::steady_clock::now();

    // Fail if there are no keyframes after tracking backwards (tracking got lost, tried to reinitialise, and failed)
    print_map_summary("Before bundle", *slam_->map_db_, timestamp_to_stage_and_frame, true);
    std::vector<std::shared_ptr<stella_vslam::data::keyframe>> all_keyframes;
    if (slam_->map_db_)
        all_keyframes = slam_->map_db_->get_all_keyframes();
    bool have_keyframes = !all_keyframes.empty();
    //bool have_keyframes = (slam_->map_db_->get_num_keyframes() > 0); // seems less reliable
    if (!have_keyframes) {
        spdlog::error("Map creation failed while tracking backwards from the initialisation frame");
        return false;
    }
    else
        spdlog::info("Map creation succeeded after tracking backwards from the initialisation frame");

    // Wait for map optimisation after loop closing to finish
    while (slam_->loop_BA_is_running() || !slam_->mapping_module_is_enabled()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const auto tp_after_backward_mapping_and_loop_closing = std::chrono::steady_clock::now();

    if (set_stage_description_)
        set_stage_description_("Optimising");

    // Optimise the map
    if (true) {
        if (!slam_->loop_detector_is_enabled())
            slam_->enable_loop_detector();

        spdlog::info("### [{}] solver 1", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
        slam_->run_loop_BA();
        spdlog::info("### [{}] solver 2", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
        // Wait for map optimisation to finish
        while (slam_->loop_BA_is_running() || !slam_->mapping_module_is_enabled()) {
            spdlog::info("### [{}] solver 3", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            spdlog::info("### [{}] solver 4", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
        }
        spdlog::info("### [{}] solver 5", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
    }

    const auto tp_after_bundle_adjust = std::chrono::steady_clock::now();

    if (set_stage_description_)
        set_stage_description_("Tracking");

    // Track forwards (without mapping) to calculate the non-keyframe camera positions
    slam_->disable_mapping_module();
    int solved_frame_count(0), unsolved_frame_count(0);
    for (int frame = begin; frame <= end; ++frame) {
        bool got_frame = get_frame_(frame, frame_image, mask, extra_keypoints);
        if (!got_frame || frame_image.empty()) {
            ++unsolved_frame_count;
            continue;
        }

        double timestamp = next_timestamp;
        next_timestamp += timestamp_increment;
        timestamp_to_stage_and_frame[timestamp] = { 2, frame };
        metrics::get_instance()->current_frame_timestamp = timestamp;

        auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, &extra_keypoints);

        camera_pose ? ++solved_frame_count : ++unsolved_frame_count;

        // Store the camera pose
        if (camera_pose && final_solve)
            final_solve->frame_to_camera[frame] = *camera_pose;
        send_frame_data(frame, slam_->get_current_frame().camera_, camera_pose, true, frame == begin);

        double stage_progress = double(frame - begin + 1) / double(1 + end - begin);
        if (set_progress_)
            set_progress_(overall_progress_percent(stage_progress, 0.7, 0.99));
        if (cancel_ && cancel_())
            return false;
    }

    const auto tp_after_resection = std::chrono::steady_clock::now();

    if (final_solve) {
        // Store the camera intrinsics (lens properties)
        // Get the camera from the keyframes (maybe there's a better place?)
        auto keyfrms = slam_->map_db_->get_all_keyframes();
        for (const auto& keyfrm : keyfrms) {
            if (keyfrm && !keyfrm->will_be_erased() && keyfrm->camera_->model_type_ == camera::model_type_t::Perspective) {
                final_solve->camera_lens = std::make_shared<camera::perspective>(*static_cast<camera::perspective*>(keyfrm->camera_));
                break;
            }
        }

        // Store the map points
        get_world_points(final_solve->world_points, final_solve->prematched_id_to_idx);
    }

    // Store the metrics
    if (final_solve) {

        // Convert the timestamped metrics to frame numbers
        track_metrics.create_frame_metrics();

        track_metrics.calculated_focal_length_x_pixels = final_solve->camera_lens->fx_;
        track_metrics.solved_frame_count = solved_frame_count;
        track_metrics.unsolved_frame_count = unsolved_frame_count;
        track_metrics.num_points = final_solve->world_points.size();

        timings& track_timings = track_metrics.track_timings;
        track_timings.forward_mapping = std::chrono::duration_cast<std::chrono::duration<double>>(tp_after_forward_mapping - tp_start).count();
        track_timings.backward_mapping = std::chrono::duration_cast<std::chrono::duration<double>>(tp_after_backward_mapping - tp_after_forward_mapping).count();
        track_timings.loop_closing = std::chrono::duration_cast<std::chrono::duration<double>>(tp_after_backward_mapping_and_loop_closing - tp_after_backward_mapping).count();
        track_timings.optimisation = std::chrono::duration_cast<std::chrono::duration<double>>(tp_after_bundle_adjust - tp_after_backward_mapping_and_loop_closing).count();
        track_timings.tracking = std::chrono::duration_cast<std::chrono::duration<double>>(tp_after_resection - tp_after_bundle_adjust).count();

    }

    return true;
}

std::shared_ptr<stella_vslam::system> solver::system() {
    return slam_;
}

void solver::get_world_points(std::vector<Eigen::Vector3d>& world_points,
                              std::unordered_map<unsigned, unsigned>& prematched_id_to_idx) const {
    std::vector<std::shared_ptr<data::landmark>> all_landmarks = slam_->map_db_->get_all_landmarks();
    world_points.resize(all_landmarks.size());
    prematched_id_to_idx.clear();
    for (unsigned i = 0; i < all_landmarks.size(); ++i) {
        world_points[i] = all_landmarks[i]->get_pos_in_world();
        auto pmid = all_landmarks[i]->prematched_id_;
        if (pmid >= 0)
            prematched_id_to_idx[pmid] = i;
    }
}

void solver::send_frame_data(int frame,
                             const stella_vslam::camera::base* camera,
                             std::shared_ptr<Eigen::Matrix4d> camera_pose,
                             bool final_points,
                             bool send_points) const {
    if (display_frame_) {
        // Copy all current solve data, so that the calling application
        // can (potentially) process it while the solver continues to run
        auto frame_data = std::make_shared<frame_display_data>();
        frame_data->frame = frame;
        if (camera && camera_pose) {
            frame_data->solve_success = true;
            frame_data->focal_length = focal_length_x_pixels_from_camera(camera);
            frame_data->camera_pose = *camera_pose;
            frame_data->final_points = final_points;
            if (send_points)
                get_world_points(frame_data->world_points, frame_data->prematched_id_to_idx);
        }

        display_frame_(frame_data);
    }
}

} // namespace stella_vslam_bfx
