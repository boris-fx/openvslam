#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/tracking_module.h"
#include "stella_vslam/mapping_module.h"
#include "stella_vslam/global_optimization_module.h"
#include "stella_vslam/camera/camera_factory.h"
#include "stella_vslam/data/camera_database.h"
#include "stella_vslam/data/common.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/frame_observation.h"
#include "stella_vslam/data/orb_params_database.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/data/bow_database.h"
#include "stella_vslam/data/bow_vocabulary.h"
#include "stella_vslam/data/marker2d.h"
#include "stella_vslam/data/map_camera_helpers.h"
#include "stella_vslam/marker_detector/aruco.h"
#include "stella_vslam/match/stereo.h"
#include "stella_vslam/feature/orb_extractor.h"
#include "stella_vslam/io/trajectory_io.h"
#include "stella_vslam/io/map_database_io_factory.h"
#include "stella_vslam/publish/map_publisher.h"
#include "stella_vslam/publish/frame_publisher.h"
#include "stella_vslam/util/converter.h"
#include "stella_vslam/util/image_converter.h"
#include "stella_vslam/report/initialisation_debugging.h"
#include "stella_vslam/report/metrics.h"
#include "stella_vslam/feature/orb_preanalysis.h"

#include <opencv2/imgcodecs.hpp>

#include <thread>

#include <spdlog/spdlog.h>

namespace {
using namespace stella_vslam;

double get_depthmap_factor(const camera::base* camera, const stella_vslam_bfx::config_settings& settings) {
    spdlog::debug("load depthmap factor");
    double depthmap_factor = 1.0;
    if (camera->setup_type_ == camera::setup_type_t::RGBD) {
        depthmap_factor = settings.depthmap_factor_;
    }
    if (depthmap_factor < 0.) {
        throw std::runtime_error("depthmap_factor must be greater than 0");
    }
    return depthmap_factor;
}

data::bow_vocabulary * loadOrbVocabulary(std::ifstream & str)
{
    auto * bow_vocab = new fbow::Vocabulary();
    bow_vocab->fromStream(str);
    if (!bow_vocab->isValid()) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab;
        bow_vocab = nullptr;
        throw std::runtime_error("Vocabulary: invalid vocabulary");
    }
    return bow_vocab;
}

data::bow_vocabulary * loadOrbVocabulary(const std::string &vocab_file_path)
{
    spdlog::info("loading ORB vocabulary: {}", vocab_file_path);

#ifdef USE_DBOW2
    auto * bow_vocab = new data::bow_vocabulary();
    try {
        bow_vocab->loadFromBinaryFile(vocab_file_path);
    }
    catch (const std::exception&) {
        spdlog::critical("wrong path to vocabulary");
        delete bow_vocab;
        bow_vocab = nullptr;
        throw std::runtime_error("Vocabulary: invalid vocabulary");
    }
    return bow_vocab;
#else
    std::ifstream file(vocab_file_path,std::ios::binary);
    if (!file)
        throw std::runtime_error("Vocabulary::readFromFile could not open: "+ vocab_file_path);
    return loadOrbVocabulary(file);
#endif
}

} // namespace

namespace stella_vslam {

system::system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path)
    : cfg_(cfg),
      use_orb_features_(cfg->settings_.use_orb_features_),
      undistort_prematches_(cfg->settings_.undistort_prematches_) {
    spdlog::debug("CONSTRUCT: system");

    bow_vocab_ = loadOrbVocabulary(vocab_file_path);

    init(cfg_.get());
}

system::system(const std::shared_ptr<config>& cfg, std::ifstream & vocab_data)
    : cfg_(cfg),
    use_orb_features_(cfg->settings_.use_orb_features_),
    undistort_prematches_(cfg->settings_.undistort_prematches_) {
    spdlog::debug("CONSTRUCT: system");

    bow_vocab_ = loadOrbVocabulary(vocab_data);

    init(cfg_.get());
}

void system::init(const config * cfg)
{
    spdlog::debug("CONSTRUCT: system");
    print_info();

    // reset static data
    data::frame::reset_next_id();

    camera_ = camera::camera_factory::create(cfg_->settings_);
    orb_params_ = new feature::orb_params(cfg_->settings_);
    spdlog::info("load orb_params \"{}\"", orb_params_->name_);

    // database
    cam_db_ = new data::camera_database();
    cam_db_->add_camera(camera_);
    map_db_ = new data::map_database(cfg_->settings_.min_num_bow_matches_);
    bow_db_ = new data::bow_database(bow_vocab_);
    orb_params_db_ = new data::orb_params_database();
    orb_params_db_->add_orb_params(orb_params_);

    // frame and map publisher
    frame_publisher_ = std::make_shared<publish::frame_publisher>(cfg_, map_db_);
    map_publisher_ = std::make_shared<publish::map_publisher>(cfg_, map_db_);

    // map I/O
    map_database_io_ = io::map_database_io_factory::create(stella_vslam::io::map_format_to_string[static_cast<unsigned>(cfg_->settings_.map_format_)]);

    // tracking module
    tracker_ = new tracking_module(cfg_, camera_, map_db_, bow_vocab_, bow_db_);
    // mapping module
    mapper_ = new mapping_module(cfg_->settings_, map_db_, bow_db_, bow_vocab_);
    // global optimization module
    global_optimizer_ = new global_optimization_module(map_db_, bow_db_, bow_vocab_, cfg_->settings_, camera_->setup_type_ != camera::setup_type_t::Monocular);

    // preprocessing modules
    depthmap_factor_ = get_depthmap_factor(camera_, cfg_->settings_);
    auto mask_rectangles = cfg->settings_.mask_rectangles_;
    //const auto min_size = cfg->settings_.min_feature_size_;
    const auto min_size = stella_vslam_bfx::orb_feature_monitor::default_min_feature_size_for_video_size(cfg_->settings_.cols_, cfg_->settings_.rows_);
    const unsigned int target_feature_count = 2000; // todo: create a user parameter for this
    feature_monitor_ = std::make_shared<stella_vslam_bfx::orb_feature_monitor>(target_feature_count);
    spdlog::info("system - min_size: {}", min_size);
    extractor_left_ = new feature::orb_extractor(orb_params_, min_size, mask_rectangles);
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(orb_params_, min_size, mask_rectangles);
    }

    if (cfg->marker_model_) {
        if (marker_detector::aruco::is_valid()) {
            spdlog::debug("marker detection: enabled");
            marker_detector_ = new marker_detector::aruco(camera_, cfg->marker_model_);
        }
        else {
            spdlog::warn("Valid marker_detector is not installed");
        }
    }

    // connect modules each other
    tracker_->set_mapping_module(mapper_);
    tracker_->set_global_optimization_module(global_optimizer_);
    mapper_->set_tracking_module(tracker_);
    mapper_->set_global_optimization_module(global_optimizer_);
    global_optimizer_->set_tracking_module(tracker_);
    global_optimizer_->set_mapping_module(mapper_);
}

system::~system() {
    global_optimization_thread_.reset(nullptr);
    delete global_optimizer_;
    global_optimizer_ = nullptr;

    mapping_thread_.reset(nullptr);
    delete mapper_;
    mapper_ = nullptr;

    delete tracker_;
    tracker_ = nullptr;

    delete bow_db_;
    bow_db_ = nullptr;
    delete map_db_;
    map_db_ = nullptr;
    delete cam_db_;
    cam_db_ = nullptr;
    delete bow_vocab_;
    bow_vocab_ = nullptr;

    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;

    delete marker_detector_;
    marker_detector_ = nullptr;

    delete orb_params_db_;
    orb_params_db_ = nullptr;

    spdlog::debug("DESTRUCT: system");
}

void system::print_info() {
    std::ostringstream message_stream;

    message_stream << std::endl;
    message_stream << "original version of OpenVSLAM," << std::endl;
    message_stream << "Copyright (C) 2019," << std::endl;
    message_stream << "National Institute of Advanced Industrial Science and Technology (AIST)" << std::endl;
    message_stream << "All rights reserved." << std::endl;
    message_stream << "stella_vslam (the changes after forking from OpenVSLAM)," << std::endl;
    message_stream << "Copyright (C) 2022, stella-cv, All rights reserved." << std::endl;
    message_stream << std::endl;
    message_stream << "This is free software," << std::endl;
    message_stream << "and you are welcome to redistribute it under certain conditions." << std::endl;
    message_stream << "See the LICENSE file." << std::endl;
    message_stream << std::endl;

    // show configuration
    message_stream << *cfg_ << std::endl;

    spdlog::info(message_stream.str());
}

void system::startup(const bool need_initialize) {
    spdlog::info("startup SLAM system");
    system_is_running_ = true;

    if (!need_initialize) {
        tracker_->tracking_state_ = tracker_state_t::Lost;
    }

    mapping_thread_ = std::make_unique<std::thread>(&stella_vslam::mapping_module::run, mapper_);
    global_optimization_thread_ = std::make_unique<std::thread>(&stella_vslam::global_optimization_module::run, global_optimizer_);
}

void system::shutdown() {
    // terminate the other threads
    auto future_mapper_terminate = mapper_->async_terminate();
    auto future_global_optimizer_terminate = global_optimizer_->async_terminate();
    future_mapper_terminate.get();
    future_global_optimizer_terminate.get();

    // wait until the threads stop
    mapping_thread_->join();
    global_optimization_thread_->join();

    spdlog::info("shutdown SLAM system");
    system_is_running_ = false;
}

void system::save_frame_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_frame_trajectory(path, format);
    resume_other_threads();
}

void system::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_keyframe_trajectory(path, format);
    resume_other_threads();
}

bool system::load_map_database(const std::string& path) const {
    pause_other_threads();
    spdlog::debug("load_map_database: {}", path);
    bool ok = map_database_io_->load(path, cam_db_, orb_params_db_, map_db_, bow_db_, bow_vocab_);
    resume_other_threads();
    return ok;
}

bool system::save_map_database(const std::string& path) const {
    pause_other_threads();
    spdlog::debug("save_map_database: {}", path);
    bool ok = map_database_io_->save(path, cam_db_, orb_params_db_, map_db_);
    resume_other_threads();
    return ok;
}

bool system::load_map_database_from_memory(std::vector<unsigned char> const& memory) const {
    pause_other_threads();
    spdlog::debug("load_map_database_from_memory: {}", memory.size());
    bool ok = map_database_io_->load_from_mem(memory, cam_db_, orb_params_db_, map_db_, bow_db_, bow_vocab_);
    resume_other_threads();
    return ok;
}

bool system::save_map_database_to_memory(std::vector<unsigned char> & memory) const {
    pause_other_threads();
    spdlog::debug("save_map_database_to_memory: {}", memory.size());
    bool ok = map_database_io_->save_to_mem(memory, cam_db_, orb_params_db_, map_db_);
    resume_other_threads();
    return ok;
}

const std::shared_ptr<publish::map_publisher> system::get_map_publisher() const {
    return map_publisher_;
}

const std::shared_ptr<publish::frame_publisher> system::get_frame_publisher() const {
    return frame_publisher_;
}

const data::frame& system::get_current_frame() const
{
    return tracker_->curr_frm_;
}

double system::get_first_map_keyframe_timestamp() const
{
    if (map_db_) {
        auto keyfrms = map_db_->get_all_keyframes();
        if (!keyfrms.empty()) {
            std::shared_ptr<stella_vslam::data::keyframe> first_keyframe = *std::min_element(keyfrms.begin(), keyfrms.end(),
               [](const auto& a, const auto& b) { return a->timestamp_ < b->timestamp_; });
            return first_keyframe->timestamp_;
        }
    }
    return -1.0;
}

bool system::relocalize_by_first_map_keyframe_pose() {
    if (map_db_) {
        auto keyfrms = map_db_->get_all_keyframes();
        if (!keyfrms.empty()) {
            std::shared_ptr<stella_vslam::data::keyframe> first_keyframe = *std::min_element(keyfrms.begin(), keyfrms.end(),
                                                                                             [](const auto& a, const auto& b) { return a->timestamp_ < b->timestamp_; });
            relocalize_by_pose(first_keyframe->get_pose_wc());
            return true;
        }
    }
    return false;
}

double system::focal_length_x_pixels() const {
    if (map_db_) {
        auto keyfrms = map_db_->get_all_keyframes();
        stella_vslam::camera::base* camera = stella_vslam_bfx::camera_from_keyframes(keyfrms);
        if (camera)
            return stella_vslam_bfx::focal_length_x_pixels_from_camera(camera);
    }
    return -1.0;
}

void system::enable_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        spdlog::critical("please call system::enable_mapping_module() after system::startup()");
    }
    // resume the mapping module
    mapper_->resume();
}

void system::disable_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        spdlog::critical("please call system::disable_mapping_module() after system::startup()");
    }
    // pause the mapping module
    auto future_pause = mapper_->async_pause();
    // wait until it stops
    future_pause.get();
}

bool system::mapping_module_is_enabled() const {
    return !mapper_->is_paused();
}

void system::enable_map_reinitialisation(std::optional<bool> always_enabled)
{
    if (always_enabled.has_value()) {
        tracker_->map_selector_.enabled = true;
        tracker_->map_selector_.allow_reset = always_enabled.value();
    }
    else
        tracker_->map_selector_.enabled = false;
}

void system::enable_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    global_optimizer_->enable_loop_detector();
}

void system::disable_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    global_optimizer_->disable_loop_detector();
}

bool system::loop_detector_is_enabled() const {
    return global_optimizer_->loop_detector_is_enabled();
}

bool system::request_loop_closure(int keyfrm1_id, int keyfrm2_id) {
    return global_optimizer_->request_loop_closure(keyfrm1_id, keyfrm2_id);
}

bool system::loop_BA_is_running() const {
    return global_optimizer_->loop_BA_is_running();
}

void system::run_loop_BA() {
    spdlog::info("### system[{}] 1", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
    global_optimizer_->run_loop_BA();
    spdlog::info("### system[{}] 1", stella_vslam_bfx::thread_dubugging::get_instance()->thread_name());
}

void system::abort_loop_BA() {
    global_optimizer_->abort_loop_BA();
}

void system::store_prematched_points(const stella_vslam_bfx::prematched_points* extra_keypoints,
                        std::vector<cv::KeyPoint>& keypts, data::frame_observation& frm_obs) const {
    if (extra_keypoints && !extra_keypoints->first.empty() && extra_keypoints->first.size() == extra_keypoints->second.size()) {
        unsigned num_prematched = extra_keypoints->first.size();
        frm_obs.prematched_keypts_.first = keypts.size();
        keypts.insert(keypts.end(), extra_keypoints->first.begin(), extra_keypoints->first.end());
        frm_obs.prematched_keypts_.second = keypts.size();
        assert(frm_obs.prematched_keypts_.second == frm_obs.prematched_keypts_.first + (int)num_prematched);

        // Create dummy ORB descriptors - these aren't used but need to be created to
        // keep data structures in sync
        if (frm_obs.descriptors_.empty())
            frm_obs.descriptors_.create(num_prematched, 32, CV_8U);
        else if (use_orb_features_)
            frm_obs.descriptors_.resize(keypts.size());
        
        if (!frm_obs.descriptors_.empty()) {
            cv::Mat extra_descriptors = frm_obs.descriptors_.rowRange(frm_obs.prematched_keypts_.first, frm_obs.prematched_keypts_.second);
            for (unsigned i = 0; i < num_prematched; ++i) {
                for (unsigned j = 0; j < 32; ++j)
                    extra_descriptors.ptr(i)[j] = static_cast<uchar>(0);
            }
        }

        // Save prematched point IDs against indices in keypts for matching (both ways)
        frm_obs.prematched_idx_to_id_.clear();
        frm_obs.prematched_id_to_idx_.clear();

        for (unsigned i = 0; i < num_prematched; ++i) {
            unsigned keypt_idx = i + frm_obs.prematched_keypts_.first, id = extra_keypoints->second[i];
            frm_obs.prematched_idx_to_id_[keypt_idx] = id;
            frm_obs.prematched_id_to_idx_[id] = keypt_idx;
        }
    }
}

data::frame system::create_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask,
                                            const stella_vslam_bfx::prematched_points* extra_keypoints) {
    // color conversion
    if (!camera_->is_valid_shape(img)) {
        spdlog::warn("preprocess: Input image size is invalid");
    }
    cv::Mat img_gray = img;
    util::convert_to_grayscale(img_gray, camera_->color_order_);

    data::frame_observation frm_obs;

    keypts_.clear();
    if (use_orb_features_) {
        // Extract ORB feature
        feature_monitor_->update_feature_extractor(img_gray, mask, extractor_left_);
        extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
        feature_monitor_->record_extraction_result(keypts_.size(), extractor_left_);
    }
    // Add the prematched points to the input vector for undistorting
    if (undistort_prematches_)
        store_prematched_points(extra_keypoints, keypts_, frm_obs);
    frm_obs.num_keypts_ = keypts_.size();
    if (keypts_.empty()) {
        spdlog::warn("preprocess: cannot extract any keypoints");
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Add already-undistorted prematched points directly
    if (!undistort_prematches_) {
        store_prematched_points(extra_keypoints, frm_obs.undist_keypts_, frm_obs);

        // keypts_ is not used for the solve but is used for viewing (points should
        // technically be re-distorted, but we will be using a different viewer anyway)
        if ( 0 <= frm_obs.prematched_keypts_.first &&
                0 <= frm_obs.prematched_keypts_.second ) {
            if ( keypts_.empty() ) {
                keypts_ = frm_obs.undist_keypts_;
            }
            else {
                keypts_.insert(keypts_.end(),
                    frm_obs.undist_keypts_.begin() + frm_obs.prematched_keypts_.first,
                    frm_obs.undist_keypts_.begin() + frm_obs.prematched_keypts_.second);
            }
        }
    }

    if (keypts_.empty()) {
        if (!use_orb_features_)
            frm_obs.descriptors_.release();
        spdlog::warn("preprocesss: cannot extract any keypoints");
    }
    frm_obs.num_keypts_ = keypts_.size();

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_);

    // Detect marker
    std::unordered_map<unsigned int, data::marker2d> markers_2d;
    if (marker_detector_) {
        marker_detector_->detect(img_gray, markers_2d);
    }

    return data::frame(timestamp, camera_, orb_params_, frm_obs, std::move(markers_2d));
}

data::frame system::create_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask) {
    // color conversion
    if (!camera_->is_valid_shape(left_img)) {
        spdlog::warn("preprocess: Input image size is invalid");
    }
    if (!camera_->is_valid_shape(right_img)) {
        spdlog::warn("preprocess: Input image size is invalid");
    }
    cv::Mat img_gray = left_img;
    cv::Mat right_img_gray = right_img;
    util::convert_to_grayscale(img_gray, camera_->color_order_);
    util::convert_to_grayscale(right_img_gray, camera_->color_order_);

    data::frame_observation frm_obs;
    //! keypoints of stereo right image
    std::vector<cv::KeyPoint> keypts_right;
    //! ORB descriptors of stereo right image
    cv::Mat descriptors_right;

    // Extract ORB feature
    keypts_.clear();
    std::thread thread_left([this, &frm_obs, &img_gray, &mask]() {
        extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
    });
    std::thread thread_right([this, &frm_obs, &right_img_gray, &mask, &keypts_right, &descriptors_right]() {
        extractor_right_->extract(right_img_gray, mask, keypts_right, descriptors_right);
    });
    thread_left.join();
    thread_right.join();
    frm_obs.num_keypts_ = keypts_.size();
    if (keypts_.empty()) {
        spdlog::warn("preprocess: cannot extract any keypoints");
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Estimate depth with stereo match
    match::stereo stereo_matcher(extractor_left_->image_pyramid_, extractor_right_->image_pyramid_,
                                 keypts_, keypts_right, frm_obs.descriptors_, descriptors_right,
                                 orb_params_->scale_factors_, orb_params_->inv_scale_factors_,
                                 camera_->focal_x_baseline_, camera_->true_baseline_);
    stereo_matcher.compute(frm_obs.stereo_x_right_, frm_obs.depths_);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_);

    // Detect marker
    std::unordered_map<unsigned int, data::marker2d> markers_2d;
    if (marker_detector_) {
        marker_detector_->detect(img_gray, markers_2d);
    }

    return data::frame(timestamp, camera_, orb_params_, frm_obs, std::move(markers_2d));
}

data::frame system::create_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask) {
    // color and depth scale conversion
    if (!camera_->is_valid_shape(rgb_img)) {
        spdlog::warn("preprocess: Input image size is invalid");
    }
    if (!camera_->is_valid_shape(depthmap)) {
        spdlog::warn("preprocess: Input image size is invalid");
    }
    cv::Mat img_gray = rgb_img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray, camera_->color_order_);
    util::convert_to_true_depth(img_depth, depthmap_factor_);

    data::frame_observation frm_obs;

    // Extract ORB feature
    keypts_.clear();
    extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
    frm_obs.num_keypts_ = keypts_.size();
    if (keypts_.empty()) {
        spdlog::warn("preprocess: cannot extract any keypoints");
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Calculate disparity from depth
    // Initialize with invalid value
    frm_obs.stereo_x_right_ = std::vector<float>(frm_obs.num_keypts_, -1);
    frm_obs.depths_ = std::vector<float>(frm_obs.num_keypts_, -1);

    for (unsigned int idx = 0; idx < frm_obs.num_keypts_; idx++) {
        const auto& keypt = keypts_.at(idx);
        const auto& undist_keypt = frm_obs.undist_keypts_.at(idx);

        const float x = keypt.pt.x;
        const float y = keypt.pt.y;

        const float depth = img_depth.at<float>(y, x);

        if (depth <= 0) {
            continue;
        }

        frm_obs.depths_.at(idx) = depth;
        frm_obs.stereo_x_right_.at(idx) = undist_keypt.pt.x - camera_->focal_x_baseline_ / depth;
    }

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_);

    // Detect marker
    std::unordered_map<unsigned int, data::marker2d> markers_2d;
    if (marker_detector_) {
        marker_detector_->detect(img_gray, markers_2d);
    }

    return data::frame(timestamp, camera_, orb_params_, frm_obs, std::move(markers_2d));
}

std::shared_ptr<Mat44_t> system::feed_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask,
                                                        const stella_vslam_bfx::prematched_points* extra_keypoints) {
    assert(camera_->setup_type_ == camera::setup_type_t::Monocular);
    if (img.empty()) {
        spdlog::warn("preprocess: empty image");
        return nullptr;
    }
    return feed_frame(create_monocular_frame(img, timestamp, mask, extra_keypoints), img);
}

std::shared_ptr<Mat44_t> system::feed_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::Stereo);
    if (left_img.empty() || right_img.empty()) {
        spdlog::warn("preprocess: empty image");
        return nullptr;
    }
    return feed_frame(create_stereo_frame(left_img, right_img, timestamp, mask), left_img);
}

std::shared_ptr<Mat44_t> system::feed_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask) {
    assert(camera_->setup_type_ == camera::setup_type_t::RGBD);
    if (rgb_img.empty() || depthmap.empty()) {
        spdlog::warn("preprocess: empty image");
        return nullptr;
    }
    return feed_frame(create_RGBD_frame(rgb_img, depthmap, timestamp, mask), rgb_img);
}

std::shared_ptr<Mat44_t> system::feed_frame(const data::frame& frm, const cv::Mat& img) {
    check_reset_request();

    const auto start = std::chrono::system_clock::now();

    const auto cam_pose_wc = tracker_->feed_frame(frm);

    const auto end = std::chrono::system_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    frame_publisher_->update(tracker_->curr_frm_.get_landmarks(),
                             !mapper_->is_paused(),
                             tracker_->tracking_state_,
                             keypts_,
                             img,
                             elapsed_ms);
    if (tracker_->tracking_state_ == tracker_state_t::Tracking && cam_pose_wc) {
        map_publisher_->set_current_cam_pose(util::converter::inverse_pose(*cam_pose_wc));
    }

    return cam_pose_wc;
}

bool system::relocalize_by_pose(const Mat44_t& cam_pose_wc) {
    const Mat44_t cam_pose_cw = util::converter::inverse_pose(cam_pose_wc);
    bool status = tracker_->request_relocalize_by_pose(cam_pose_cw);
    if (status) {
        // Even if state will be lost, still update the pose in map_publisher_
        // to clearly show new camera position
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }
    return status;
}

bool system::relocalize_by_pose_2d(const Mat44_t& cam_pose_wc, const Vec3_t& normal_vector) {
    const Mat44_t cam_pose_cw = util::converter::inverse_pose(cam_pose_wc);
    bool status = tracker_->request_relocalize_by_pose_2d(cam_pose_cw, normal_vector);
    if (status) {
        // Even if state will be lost, still update the pose in map_publisher_
        // to clearly show new camera position
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }
    return status;
}

void system::pause_tracker() {
    auto future_pause = tracker_->async_pause();
    future_pause.get();
}

bool system::tracker_is_paused() const {
    return tracker_->is_paused();
}

void system::resume_tracker() {
    tracker_->resume();
}

void system::request_reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    reset_is_requested_ = true;
}

bool system::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void system::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool system::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

camera::base* system::get_camera() const {
    return camera_;
}

void system::check_reset_request() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (reset_is_requested_) {
        tracker_->reset();
        reset_is_requested_ = false;
    }
}

void system::pause_other_threads() const {
    // pause the mapping module
    if (mapper_ && !mapper_->is_terminated()) {
        auto future_pause = mapper_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout) {
            if (mapper_->is_terminated()) {
                break;
            }
        }
    }
    // pause the global optimization module
    if (global_optimizer_ && !global_optimizer_->is_terminated()) {
        auto future_pause = global_optimizer_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout) {
            if (global_optimizer_->is_terminated()) {
                break;
            }
        }
    }
}

void system::resume_other_threads() const {
    // resume the global optimization module
    if (global_optimizer_) {
        global_optimizer_->resume();
    }
    // resume the mapping module
    if (mapper_) {
        mapper_->resume();
    }
}

bool system::extractor_boost_check(std::vector<cv::KeyPoint> const& keypts) {
    if (extractor_boost_checked_)
        return false;
    extractor_boost_checked_ = true;

    // Count keypoints for which matching will be attempted in match::area::match_in_consistent_area(), i.e. those at the 0-th scale
    int matchable_count = std::count_if(keypts.begin(), keypts.end(), [](cv::KeyPoint const& keypt) { return keypt.octave == 0; });
    bool boost(matchable_count < 500);

    float min_size_boost(1);
    if (boost) {
        min_size_boost = 0.5f;
        spdlog::info("system - applying min size boost of {}", min_size_boost);
        if (extractor_left_)
            extractor_left_->min_size_multiplier_ = min_size_boost;
        if (extractor_right_)
            extractor_right_->min_size_multiplier_ = min_size_boost;
    }

    stella_vslam_bfx::metrics::get_instance()->feature_min_size_scale = min_size_boost;

    return boost;
}

void system::boost_extractors(float boost) {

    spdlog::info("system - applying min size boost of {}", boost);
    stella_vslam_bfx::metrics::get_instance()->feature_min_size_scale = boost;

    if (extractor_left_)
        extractor_left_->min_size_multiplier_ = boost;
    if (extractor_right_)
        extractor_right_->min_size_multiplier_ = boost;
}




} // namespace stella_vslam
