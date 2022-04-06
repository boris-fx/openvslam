#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/util/yaml.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

// TODO move to shared location for other examples to use
stella_vslam_bfx::config_settings * settings_from_yaml(YAML::Node yaml_node)
{
    stella_vslam_bfx::config_settings * settings = nullptr;

    // Essential camera settings
    const auto camera_node = yaml_node["Camera"];

    stella_vslam::camera::model_type_t camera_model;
    const auto model_str = camera_node["model"].as<std::string>();
    if (model_str == "perspective")
        camera_model = stella_vslam::camera::model_type_t::Perspective;
    else if (model_str == "fisheye")
        camera_model = stella_vslam::camera::model_type_t::Fisheye;
    else if (model_str == "equirectangular")
        camera_model = stella_vslam::camera::model_type_t::Equirectangular;
    else if (model_str == "radial_division")
        camera_model = stella_vslam::camera::model_type_t::RadialDivision;
    else
        throw std::runtime_error("Invalid camera model: " + model_str);

    stella_vslam::camera::setup_type_t camera_setup;
    const auto setup_type_str = camera_node["setup"].as<std::string>();
    if (setup_type_str == "monocular")
        camera_setup = stella_vslam::camera::setup_type_t::Monocular;
    else if (setup_type_str == "stereo")
        camera_setup = stella_vslam::camera::setup_type_t::Stereo;
    else if (setup_type_str == "RGBD")
        camera_setup = stella_vslam::camera::setup_type_t::RGBD;
    else
        throw std::runtime_error("Invalid setup type: " + setup_type_str);
    
    stella_vslam::camera::color_order_t colour_order = stella_vslam::camera::color_order_t::Gray;
    if (camera_node["color_order"])
    {
        const auto colour_order_str = camera_node["color_order"].as<std::string>();
        if (colour_order_str == "Gray")
            colour_order = stella_vslam::camera::color_order_t::Gray;
        else if (colour_order_str == "RGB" || colour_order_str == "RGBA")
            colour_order = stella_vslam::camera::color_order_t::RGB;
        else if (colour_order_str == "BGR" || colour_order_str == "BGRA")
            colour_order = stella_vslam::camera::color_order_t::BGR;
        else
            throw std::runtime_error("Invalid color order: " + colour_order_str);
    }
    
    int cols = camera_node["cols"].as<unsigned int>();
    int rows = camera_node["rows"].as<unsigned int>();
    double fps = camera_node["fps"].as<double>();
    
    switch (camera_model)
    {
        case stella_vslam::camera::model_type_t::Perspective:
        {
            double fx = camera_node["fx"].as<double>();
            double fy = camera_node["fy"].as<double>();
            double cx = camera_node["cx"].as<double>();
            double cy = camera_node["cy"].as<double>();
            double p1 = camera_node["p1"].as<double>();
            double p2 = camera_node["p2"].as<double>();
            double k1 = camera_node["k1"].as<double>();
            double k2 = camera_node["k2"].as<double>();
            double k3 = camera_node["k3"].as<double>();
            
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                            colour_order, cols, rows, fps,
                                            fx, fy, cx, cy, p1, p2, k1, k2, k3);
            break;
        }
        case stella_vslam::camera::model_type_t::Fisheye:
        {
            double fx = camera_node["fx"].as<double>();
            double fy = camera_node["fy"].as<double>();
            double cx = camera_node["cx"].as<double>();
            double cy = camera_node["cy"].as<double>();
            double k1 = camera_node["k1"].as<double>();
            double k2 = camera_node["k2"].as<double>();
            double k3 = camera_node["k3"].as<double>();
            double k4 = camera_node["k4"].as<double>();
            
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                            colour_order, cols, rows, fps,
                                            fx, fy, cx, cy, k1, k2, k3, k4);
            break;
        }
        case stella_vslam::camera::model_type_t::RadialDivision:
        {
            double fx = camera_node["fx"].as<double>();
            double fy = camera_node["fy"].as<double>();
            double cx = camera_node["cx"].as<double>();
            double cy = camera_node["cy"].as<double>();
            double d = camera_node["distortion"].as<double>();
            
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                            colour_order, cols, rows, fps,
                                            fx, fy, cx, cy, d);
            break;
        }
        case stella_vslam::camera::model_type_t::Equirectangular:
        {
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                            colour_order, cols, rows, fps);
            break;
        }
        default:
            break;
    }
    
    if (!settings)
        throw std::runtime_error("Invalid settings");
    
    // Stereo settings
    settings->focal_x_baseline_ = camera_node["focal_x_baseline"].as<double>(0.0);
    settings->depth_threshold_ = camera_node["depth_threshold"].as<double>(40.0);

    // Optional settings //
    using namespace stella_vslam::util;

    // Preprocessing
    const auto preproc_node = yaml_optional_ref(yaml_node, "Preprocessing");
    if ( preproc_node.size() ) {
        settings->max_num_keypoints_ = preproc_node["max_num_keypoints"].as<unsigned int>(2000);
        settings->ini_max_num_keypoints_ = preproc_node["ini_max_num_keypoints"].as<unsigned int>(2 * settings->max_num_keypoints_);
        settings->depthmap_factor_ = preproc_node["depthmap_factor"].as<double>(1.0);
        settings->mask_rectangles_ =
            preproc_node["mask_rectangles"].as<std::vector<std::vector<float>>>(std::vector<std::vector<float>>());
    }

    // ORB
    const auto orb_node = yaml_optional_ref(yaml_node, "Feature");
    if ( orb_node.size() ) {
        settings->ini_fast_threshold_ = orb_node["ini_fast_threshold"].as<unsigned int>(20);
        settings->min_fast_threshold_ = orb_node["min_fast_threshold"].as<unsigned int>(7);
        settings->num_levels_ = orb_node["num_levels"].as<unsigned int>(8);
        settings->scale_factor_ = orb_node["scale_factor"].as<float>(1.2);
    }

    // Marker
    const auto marker_node = yaml_optional_ref(yaml_node, "MarkerModel");
    if ( marker_node.size() ) {
        settings->marker_model_is_enabled_ = true;
        const auto marker_str = marker_node["type"].as<std::string>();
        if (marker_str == "aruco") {
            settings->marker_model_ = stella_vslam::marker_model::model_type_t::Aruco;
            settings->marker_width_ = marker_node["width"].as<double>();
            settings->marker_size_ = marker_node["marker_size"].as<double>();
            settings->max_markers_ = marker_node["max_markers"].as<double>();
        }
        else
            throw std::runtime_error("Invalid marker model type :" + marker_str);
    }

    // Tracking
    const auto track_node = yaml_optional_ref(yaml_node, "Tracking");
    if ( track_node.size() ) {
        settings->enable_auto_relocalization_ = track_node["enable_auto_relocalization"].as<bool>(true);
        settings->use_robust_matcher_for_relocalization_request_ = track_node["use_robust_matcher_for_relocalization_request"].as<bool>(false);
        settings->reloc_distance_threshold_ = track_node["reloc_distance_threshold"].as<double>(0.2);
        settings->reloc_angle_threshold_ = track_node["reloc_angle_threshold"].as<double>(0.45);
    }

    // Mapping
    const auto mapping_node = yaml_optional_ref(yaml_node, "Mapping");
    if ( mapping_node.size() ) {
        if (mapping_node["baseline_dist_thr"]) {
            if (mapping_node["baseline_dist_thr_ratio"]) {
                throw std::runtime_error("Do not set both baseline_dist_thr_ratio and baseline_dist_thr.");
            }
            settings->baseline_dist_thr_ = mapping_node["baseline_dist_thr"].as<double>(1.0);
            settings->use_baseline_dist_thr_ratio_ = false;
        }
        else {
            settings->baseline_dist_thr_ratio_ = mapping_node["baseline_dist_thr_ratio"].as<double>(0.02);
            settings->use_baseline_dist_thr_ratio_ = true;
        }
        settings->num_obs_thr_ = mapping_node["num_obs_thr"].as<unsigned int>(2);
        settings->num_reliable_keyfrms_ = mapping_node["num_reliable_keyfrms"].as<unsigned int>(2);
        settings->desired_valid_obs_ = mapping_node["desired_valid_obs"].as<unsigned int>(0);
        settings->num_obs_keyfrms_thr_ = mapping_node["num_obs_keyframes_thr"].as<unsigned int>(2);
        settings->redundant_obs_ratio_thr_ = mapping_node["redundant_obs_ratio_thr"].as<double>(0.9);
        settings->observed_ratio_thr_ = mapping_node["observed_ratio_thr"].as<double>(0.3);
    }

    // Keyframe
    const auto keyframe_node = yaml_optional_ref(yaml_node, "KeyframeInserter");
    if ( keyframe_node.size() ) {
        settings->enough_lms_thr_ = keyframe_node["enough_lms_thr"].as<unsigned int>(100);
        settings->max_interval_ = keyframe_node["max_interval"].as<double>(1.0);
        settings->min_interval_ = keyframe_node["min_interval"].as<double>(0.1);
        settings->max_distance_ = keyframe_node["max_distance"].as<double>(-1.0);
        settings->lms_ratio_thr_almost_all_lms_are_tracked_ = keyframe_node["lms_ratio_thr_almost_all_lms_are_tracked"].as<double>(0.95);
        settings->lms_ratio_thr_view_changed_ = keyframe_node["lms_ratio_thr_view_changed"].as<double>(0.9);
    }

    // Solve/initializer
    const auto solve_node = yaml_optional_ref(yaml_node, "Initializer");
    if ( solve_node.size() ) {
        settings->use_fixed_seed_= solve_node["use_fixed_seed"].as<bool>(true);
        settings->num_ransac_iterations_= solve_node["num_ransac_iterations"].as<unsigned int>(100);
        settings->min_num_triangulated_pts_ = solve_node["num_min_triangulated_pts"].as<unsigned int>(50);
        settings->num_ba_iterations_= solve_node["num_ba_iterations"].as<unsigned int>(20);
        settings->parallax_deg_threshold_= solve_node["parallax_deg_threshold"].as<float>(1.0);
        settings->reprojection_error_threshold_= solve_node["reprojection_error_threshold"].as<float>(4.0);
        settings->scaling_factor_= solve_node["scaling_factor"].as<float>(1.0);
    }
    
    // Loop detector
    const auto loop_node = yaml_optional_ref(yaml_node, "LoopDetector");
    if ( loop_node.size() ) {
        settings->loop_detector_is_enabled_= loop_node["enabled"].as<bool>(true);
        settings->reject_by_graph_distance_ = loop_node["reject_by_graph_distance"].as<bool>(false);
        settings->num_final_matches_threshold_= loop_node["num_final_matches_threshold"].as<unsigned int>(40);
        settings->min_continuity_= loop_node["min_continuity"].as<unsigned int>(3);
        settings->min_distance_on_graph_ = loop_node["min_distance_on_graph"].as<unsigned int>(50);
        settings->num_matches_thr_ = loop_node["num_matches_thr"].as<unsigned int>(20);
        settings->num_matches_thr_robust_matcher_ = loop_node["num_matches_thr_robust_matcher"].as<unsigned int>(0);
        settings->num_optimized_inliers_thr_ = loop_node["num_optimized_inliers_thr"].as<unsigned int>(20);
        settings->top_n_covisibilities_to_search_ = loop_node["top_n_covisibilities_to_search"].as<unsigned int>(0);
    }

    // Relocalizer
    const auto reloc_node = yaml_optional_ref(yaml_node, "Relocalizer");
    if ( reloc_node.size() ) {
        settings->min_num_bow_matches_ = reloc_node["min_num_bow_matches"].as<unsigned int>(20);
        settings->min_num_valid_obs_=  reloc_node["min_num_valid_obs"].as<unsigned int>(50);
        settings->bow_match_lowe_ratio_ = reloc_node["bow_match_lowe_ratio"].as<double>(0.75);
        settings->proj_match_lowe_ratio_ = reloc_node["proj_match_lowe_ratio"].as<double>(0.9);
        settings->robust_match_lowe_ratio_ = reloc_node["robust_match_lowe_ratio"].as<double>(0.8);
    }

    // Stereo rectifier
    const auto stereo_node = yaml_optional_ref(yaml_node, "StereoRectifier");
    if ( stereo_node.size() ) {
        settings->K_left_ = stereo_node["K_left"].as<std::vector<double>>();
        settings->K_right_ = stereo_node["K_right"].as<std::vector<double>>();
        settings->R_left_ = stereo_node["R_left"].as<std::vector<double>>();
        settings->R_right_ = stereo_node["R_right"].as<std::vector<double>>();
        settings->D_left_ = stereo_node["D_left"].as<std::vector<double>>();
        settings->D_right_ = stereo_node["D_right"].as<std::vector<double>>();
    }

    // System
    const auto system_node = yaml_optional_ref(yaml_node, "System");
    if ( system_node.size() ) {
        const auto map_format_str = system_node["map_format"].as<std::string>();
        if (map_format_str == "sqlite3")
            settings->map_format_ = stella_vslam::io::map_format_t::SQLite3;
        else
            settings->map_format_ = stella_vslam::io::map_format_t::Msgpack;
    }

    std::cout << settings << std::endl;

    return settings;
}

void mono_tracking(const std::shared_ptr<stella_vslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& video_file_path, const std::string& mask_img_path,
                   const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                   const bool eval_log, const std::string& map_db_path, YAML::Node yaml_node) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    stella_vslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        stella_vslam::util::yaml_optional_ref(yaml_node, "PangolinViewer"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        stella_vslam::util::yaml_optional_ref(yaml_node, "SocketPublisher"), &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());

#endif

    auto video = cv::VideoCapture(video_file_path, cv::CAP_FFMPEG);
    std::vector<double> track_times;

    cv::Mat frame;
    double timestamp = 0.0;

    unsigned int num_frame = 0;

    bool is_not_end = true;
    // run the SLAM in another thread
    std::thread thread([&]() {
        while (is_not_end) {
            is_not_end = video.read(frame);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!frame.empty() && (num_frame % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                SLAM.feed_monocular_frame(frame, timestamp, mask);
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (num_frame % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep) {
                const auto wait_time = 1.0 / cfg->camera_->fps_ - track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto video_file_path = op.add<popl::Value<std::string>>("m", "video", "video file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !video_file_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration    
    YAML::Node yaml_node;
    try {
        //cfg = std::make_shared<stella_vslam::config>(config_file_path->value());
        yaml_node = YAML::LoadFile(config_file_path->value());
        
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Read settings from YAML
    auto settings = settings_from_yaml(yaml_node);
    std::shared_ptr<stella_vslam::config> cfg = std::make_shared<stella_vslam::config>(*settings);

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), video_file_path->value(), mask_img_path->value(),
                      frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
                      eval_log->is_set(), map_db_path->value(), yaml_node);
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    if (settings)
        delete settings;

    return EXIT_SUCCESS;
}
