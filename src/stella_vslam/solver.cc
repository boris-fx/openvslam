#include "solver.h"

#include <opencv2/core/mat.hpp>

#include <spdlog/spdlog.h>

#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/frame.h>
#include <stella_vslam/data/keyframe_autocalibration_wrapper.h>
#include <stella_vslam/data/map_database.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/report/plot_html.h>
#include <stella_vslam/report/metrics.h>

#include "type.h"
#include "system.h"

using namespace stella_vslam;

namespace stella_vslam_bfx {

void solve::clear() {
    frame_to_camera.clear();
    world_points.clear();
    camera_lens.reset();
}

void frame_display_data::clear() {
    frame = -1;
    final_points = false;
    solve_success = false;
    focal_length = 0.0;
    camera_pose.setIdentity();
    world_points.clear();
}

solver::solver(const std::shared_ptr<config>& cfg,
               const std::string& vocab_file_path,
               std::function<bool(int, cv::Mat&)> get_frame)
: get_frame_(get_frame) {
    spdlog::debug("debug log message test solver");
    spdlog::info("info log message test solver");
    // spdlog::set_level(spdlog::level::from_str(log_level->value()));
    // build the slam system
    slam_ = std::make_shared<stella_vslam::system>(cfg, vocab_file_path);

    spdlog::debug("debug log message test solver 2");
    spdlog::info("info log message test solver 2");

    bool already_have_map(false); // see run_video_slam for the case where we already have a map
    slam_->startup(!already_have_map);
}

#if !defined(USE_DBOW2)
solver::solver(const std::shared_ptr<config>& cfg,
               std::ifstream & vocab_data,
               std::function<bool(int, cv::Mat&)> get_frame)
: get_frame_(get_frame) {
    slam_ = std::make_shared<stella_vslam::system>(cfg, vocab_data);
    slam_->startup();
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

static std::tuple<std::shared_ptr<data::keyframe>, std::optional<int>> earliest_valid_keyframe(std::vector<std::shared_ptr<data::keyframe>> const& keyfrms,
                                                                                std::map<double, int> const& timestampToVideoFrame) {
    if (!keyfrms.empty()) {
        std::shared_ptr<data::keyframe> first_keyframe_data = *std::min_element(keyfrms.begin(), keyfrms.end(),
                                                                                [](const auto& a, const auto& b) { // earliest above landmark threshold
                                                                                    int landmarkThreshold(10);
                                                                                    bool validLandmarksA(a->get_valid_landmarks().size() >= landmarkThreshold);
                                                                                    bool validLandmarksB(b->get_valid_landmarks().size() >= landmarkThreshold);
                                                                                    if (!validLandmarksA && validLandmarksB)
                                                                                        return false;
                                                                                    if (validLandmarksA && !validLandmarksB)
                                                                                        return true;
                                                                                    return a->timestamp_ < b->timestamp_;
                                                                                });
        auto f = timestampToVideoFrame.find(first_keyframe_data->timestamp_);
        if (f != timestampToVideoFrame.end())
            return {first_keyframe_data, f->second};
    }
    return {nullptr, {}};
}

double overall_progress_percent(double stage_progress, double stage_start_progress, double stage_end_progress) {
    return 100.0 * (stage_start_progress + stage_progress * (stage_end_progress - stage_start_progress));
}

bool solver::track_frame_range(int begin, int end, tracking_direction direction, solve* final_solve) {
    cv::Mat frame_image;
    cv::Mat mask; // to do!!
    prematched_points* extra_keypoints(nullptr); // to do!!
    if (final_solve)
        final_solve->clear();

    if (!get_frame_)
        return false; // report error!

    std::map<double, int> timestampToVideoFrame;

    double timestamp(0);

    // To do, support the tracking direction
    if (direction == tracking_direction_backwards)
        return false;

    if (set_stage_description_)
        set_stage_description_("Analysing video");
    if (set_progress_)
        set_progress_(0);

    const auto tp_start = std::chrono::steady_clock::now();

    // Track forwards to create the map
    for (int frame = begin; frame <= end; ++frame) {
        bool got_frame = get_frame_(frame, frame_image);
        if (!got_frame || frame_image.empty())
            continue;

        timestamp = (double)frame;

        timestampToVideoFrame[timestamp] = frame;

        auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, extra_keypoints);
        send_frame_data(frame, slam_->get_current_frame().camera_, camera_pose, false, true);

        double stage_progress = double(frame - begin + 1) / double(1 + end - begin);
        if (set_progress_)
            set_progress_(overall_progress_percent(stage_progress, 0.0, 0.4));
        if (cancel_ && cancel_())
            return false;
    }

    const auto tp_after_forward_mapping = std::chrono::steady_clock::now();

    // Find the initialisation point (the first keyframe in the map)
    // Relocalise the tracker to the initialisation point's pose
    auto [first_keyframe_data, first_keyframe] = earliest_valid_keyframe(slam_->map_db_->get_all_keyframes(), timestampToVideoFrame);
    if (!first_keyframe_data) {
        spdlog::error("Failed to create any valid keyframes during initialialisation");
        
        metrics& track_metrics = *metrics::get_instance();
        track_metrics.create_frame_metrics(timestampToVideoFrame);
        
        return false;
    }
    bool resultRelocalise = slam_->relocalize_by_pose(first_keyframe_data->get_pose_wc());
    if (resultRelocalise == true)
        spdlog::info("Relocalised to first keyframe pose (video frame {})", first_keyframe.value());
    else {
        spdlog::error("Failed to relocalise to first keyframe pose (video frame {})", first_keyframe.value());
        return false;
    }

    // Some bookkeeping
    std::vector<double> videoFrameToTimestamp(end+1, -1.0);
    for (auto const& tf : timestampToVideoFrame)
        videoFrameToTimestamp[tf.second] = tf.first;

    if (set_stage_description_)
        set_stage_description_("Revisiting video");

    // Track backwards from the initialization point to finish the map
    for (int frame = first_keyframe.value() - 1; frame >= begin; --frame) {
        bool got_frame = get_frame_(frame, frame_image);
        if (!got_frame || frame_image.empty())
            continue;

        timestamp = videoFrameToTimestamp[frame];

        auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, extra_keypoints);
        send_frame_data(frame, slam_->get_current_frame().camera_, camera_pose, false, true);

        double stage_progress = double(first_keyframe.value() - frame) / double(first_keyframe.value() - begin);
        if (set_progress_)
            set_progress_(overall_progress_percent(stage_progress, 0.4, 0.5));
        if (cancel_ && cancel_())
            return false;
    }

    const auto tp_after_backward_mapping = std::chrono::steady_clock::now();

    // Wait for map optimisation after loop closing to finish
    while (slam_->loop_BA_is_running() || !slam_->mapping_module_is_enabled()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const auto tp_after_backward_mapping_and_loop_closing = std::chrono::steady_clock::now();

    if (set_stage_description_)
        set_stage_description_("Optimising");

    // Optimise the map
    if (true) {
        spdlog::info("### solver[{}] 1", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
        slam_->run_loop_BA();
        spdlog::info("### solver[{}] 2", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
        // Wait for map optimisation to finish
        while (slam_->loop_BA_is_running() || !slam_->mapping_module_is_enabled()) {
            spdlog::info("### solver[{}] 3", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            spdlog::info("### solver[{}] 4", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
        }
        spdlog::info("### solver[{}] 5", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
    }

    const auto tp_after_bundle_adjust = std::chrono::steady_clock::now();

    if (set_stage_description_)
        set_stage_description_("Tracking");

    // Track forwards (without mapping) to calculate the non-keyframe camera positions
    slam_->disable_mapping_module();
    int solved_frame_count(0), unsolved_frame_count(0);
    for (int frame = begin; frame <= end; ++frame) {
        bool got_frame = get_frame_(frame, frame_image);
        if (!got_frame || frame_image.empty()) {
            ++unsolved_frame_count;
            continue;
        }

        timestamp = videoFrameToTimestamp[frame];

        auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, extra_keypoints);

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
        get_world_points(final_solve->world_points);
    }

    // Store the metrics
    if (final_solve) {
        metrics& track_metrics = *metrics::get_instance();

        // Convert the timestamped metrics to frame numbers
        track_metrics.create_frame_metrics(timestampToVideoFrame);

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

void solver::get_world_points(std::vector<Eigen::Vector3d>& world_points) const {
    world_points.clear();
    std::vector<std::shared_ptr<data::landmark>> all_landmarks = slam_->map_db_->get_all_landmarks();
    std::transform(all_landmarks.cbegin(), all_landmarks.cend(), std::back_inserter(world_points),
                   [](const std::shared_ptr<data::landmark>& value) { return value->get_pos_in_world(); });
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
                get_world_points(frame_data->world_points);
        }

        display_frame_(frame_data);
    }
}

} // namespace stella_vslam_bfx
