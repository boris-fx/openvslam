#include "config_settings.h"
#include <string>

namespace stella_vslam_bfx {

config_settings::config_settings(stella_vslam::camera::model_type_t camera_model,
                                    stella_vslam::camera::setup_type_t camera_setup,
                                    stella_vslam::camera::color_order_t colour_order,
                                    int cols, int rows, double fps, double fx, double fy,
                                    double cx, double cy, double p1, double p2,
                                 double k1, double k2, double k3)
    :
    camera_model_(camera_model), camera_setup_(camera_setup), colour_order_(colour_order),
    cols_(cols), rows_(rows), fps_(fps)
{
    if (camera_model != stella_vslam::camera::model_type_t::Perspective)
        throw std::runtime_error("Incorrect camera settings for type");

    perspective_settings_.fx_ = fx;
    perspective_settings_.fy_ = fy;
    perspective_settings_.cx_ = cx;
    perspective_settings_.cy_ = cy;
    perspective_settings_.p1_ = p1;
    perspective_settings_.p2_ = p2;
    perspective_settings_.k1_ = k1;
    perspective_settings_.k2_ = k2;
    perspective_settings_.k3_ = k3;
}

config_settings::config_settings(stella_vslam::camera::model_type_t camera_model,
                                    stella_vslam::camera::setup_type_t camera_setup,
                                    stella_vslam::camera::color_order_t colour_order,
                                    int cols, int rows, double fps,
                                    double fx, double fy, double cx, double cy,
                                 double k1, double k2, double k3, double k4)
    :
    camera_model_(camera_model), camera_setup_(camera_setup), colour_order_(colour_order),
    cols_(cols), rows_(rows), fps_(fps)
{
    if (camera_model != stella_vslam::camera::model_type_t::Fisheye)
        throw std::runtime_error("Incorrect camera settings for type");

    fisheye_settings_.fx_ = fx;
    fisheye_settings_.fy_ = fy;
    fisheye_settings_.cx_ = cx;
    fisheye_settings_.cy_ = cy;
    fisheye_settings_.k1_ = k1;
    fisheye_settings_.k2_ = k2;
    fisheye_settings_.k3_ = k3;
    fisheye_settings_.k4_ = k4;
}

config_settings::config_settings(stella_vslam::camera::model_type_t camera_model,
                                    stella_vslam::camera::setup_type_t camera_setup,
                                    stella_vslam::camera::color_order_t colour_order,
                                    int cols, int rows, double fps,
                                    double fx, double fy, double cx, double cy,
                                 double distortion)
    :
    camera_model_(camera_model), camera_setup_(camera_setup), colour_order_(colour_order),
    cols_(cols), rows_(rows), fps_(fps)
{
    if (camera_model != stella_vslam::camera::model_type_t::RadialDivision)
        throw std::runtime_error("Incorrect camera settings for type");

    radial_division_settings_.fx_ = fx;
    radial_division_settings_.fy_ = fy;
    radial_division_settings_.cx_ = cx;
    radial_division_settings_.cy_ = cy;
    radial_division_settings_.distortion_ = distortion;
}

config_settings::config_settings(stella_vslam::camera::model_type_t camera_model,
                                    stella_vslam::camera::setup_type_t camera_setup,
                                    stella_vslam::camera::color_order_t colour_order,
                                    int cols, int rows, double fps) :
    camera_model_(camera_model), camera_setup_(camera_setup), colour_order_(colour_order),
    cols_(cols), rows_(rows), fps_(fps)
{
    if (camera_model != stella_vslam::camera::model_type_t::Equirectangular)
        throw std::runtime_error("Incorrect camera settings for type");
}

std::ostream& operator<<(std::ostream& os, const config_settings& settings) {
    os << "-- Config settings --\n";
    
    os << "Camera\n";
    os << "\tModel: " <<
        stella_vslam::camera::model_type_to_string[static_cast<unsigned>(settings.camera_model_)]
        << std::endl;
    os << "\tSetup: " <<
        stella_vslam::camera::setup_type_to_string[static_cast<unsigned>(settings.camera_setup_)]
        << std::endl;
    os << "\tColor order: " <<
        stella_vslam::camera::color_order_to_string[static_cast<unsigned>(settings.colour_order_)]
        << std::endl;

    os << "\tCols: " << settings.cols_ << std::endl;
    os << "\tRows: " << settings.rows_ << std::endl;
    os << "\tFps: " << settings.fps_ << std::endl;

    switch (settings.camera_model_)
    {
        case stella_vslam::camera::model_type_t::Perspective:
            os << "\tfx: " << settings.perspective_settings_.fx_ << std::endl;
            os << "\tfy: " << settings.perspective_settings_.fy_ << std::endl;
            os << "\tcx: " << settings.perspective_settings_.cx_ << std::endl;
            os << "\tcy: " << settings.perspective_settings_.cy_ << std::endl;
            os << "\tp1: " << settings.perspective_settings_.p1_ << std::endl;
            os << "\tp2: " << settings.perspective_settings_.p2_ << std::endl;
            os << "\tk1: " << settings.perspective_settings_.k1_ << std::endl;
            os << "\tk2: " << settings.perspective_settings_.k2_ << std::endl;
            os << "\tk3: " << settings.perspective_settings_.k3_ << std::endl;
            os << "\tFocal X baseline: " << settings.focal_x_baseline_ << std::endl;
            os << "\tDepth threshold: " << settings.depth_threshold_ << std::endl;
            break;
        case stella_vslam::camera::model_type_t::Fisheye:
            os << "\tfx: " << settings.fisheye_settings_.fx_ << std::endl;
            os << "\tfy: " << settings.fisheye_settings_.fy_ << std::endl;
            os << "\tcx: " << settings.fisheye_settings_.cx_ << std::endl;
            os << "\tcy: " << settings.fisheye_settings_.cy_ << std::endl;
            os << "\tk1: " << settings.fisheye_settings_.k1_ << std::endl;
            os << "\tk2: " << settings.fisheye_settings_.k2_ << std::endl;
            os << "\tk3: " << settings.fisheye_settings_.k3_ << std::endl;
            os << "\tk4: " << settings.fisheye_settings_.k4_ << std::endl;
            os << "\tFocal X baseline: " << settings.focal_x_baseline_ << std::endl;
            os << "\tDepth threshold: " << settings.depth_threshold_ << std::endl;
            break;
        case stella_vslam::camera::model_type_t::RadialDivision:
            os << "\tfx: " << settings.radial_division_settings_.fx_ << std::endl;
            os << "\tfy: " << settings.radial_division_settings_.fy_ << std::endl;
            os << "\tcx: " << settings.radial_division_settings_.cx_ << std::endl;
            os << "\tcy: " << settings.radial_division_settings_.cy_ << std::endl;
            os << "\tDistortion: " << settings.radial_division_settings_.distortion_ << std::endl;
            os << "\tFocal X baseline: " << settings.focal_x_baseline_ << std::endl;
            os << "\tDepth threshold: " << settings.depth_threshold_ << std::endl;
            break;
        default:
            break;
    }

    os << "Preprocessing\n";
    os << "\tMax num keypoints: " << settings.max_num_keypoints_ << std::endl;
    os << "\tIni max num keypoints: " << settings.ini_max_num_keypoints_ << std::endl;
    os << "\tDepthmap factor: " << settings.depthmap_factor_ << std::endl;
    os << "\tMask rectangles: ";
    for (auto rect : settings.mask_rectangles_) {
        os << "[ ";
        for (auto val : rect)
            os << val << " ";
        os << "] ";
    }
    os << std::endl;

    os << "Feature/ORB\n";
    os << "\tUse ORB features: " << settings.use_orb_features_ << std::endl;
    os << "\tUndistort prematches: " << settings.undistort_prematches_ << std::endl;
    os << "\tIni fast threshold: " << settings.ini_fast_threshold_ << std::endl;
    os << "\tMin fast threshold: " << settings.min_fast_threshold_ << std::endl;
    os << "\tNum levels: " << settings.num_levels_ << std::endl;
    os << "\tORB scale factor: " << settings.scale_factor_ << std::endl;

    os << "MarkerModel\n";
    os << "\tEnabled: " << settings.marker_model_is_enabled_ << std::endl;
    os << "\tModel: " <<
        stella_vslam::marker_model::model_type_to_string[static_cast<unsigned>(settings.marker_model_)]
        << std::endl;
    os << "\tWidth: " << settings.marker_width_ << std::endl;
    os << "\tSize: " << settings.marker_size_ << std::endl;
    os << "\tMax markers: " << settings.max_markers_ << std::endl;

    os << "Tracking\n";
    os << "\tEnable auto relocalization: " << settings.enable_auto_relocalization_ << std::endl;
    os << "\tUse robust matcher for relocalization request: " << settings.use_robust_matcher_for_relocalization_request_ << std::endl;
    os << "\tReloc distance threshold: " << settings.reloc_distance_threshold_ << std::endl;
    os << "\tReloc angle threshold: " << settings.reloc_angle_threshold_ << std::endl;

    os << "Mapping\n";
    os << "\tUse baseline dist thr ratio: " << settings.use_baseline_dist_thr_ratio_ << std::endl;
    os << "\tUse additional keyframes for monocular: " << settings.use_additional_keyframes_for_monocular_ << std::endl;
    os << "\tNum obs threshold: " << settings.num_obs_thr_ << std::endl;
    os << "\tNum reliable keyframes: " << settings.num_reliable_keyfrms_ << std::endl;
    os << "\tDesired valid observations: " << settings.desired_valid_obs_ << std::endl;
    os << "\tBaseline dist thr: " << settings.baseline_dist_thr_ << std::endl;
    os << "\tBaseline dist thr ratio: " << settings.baseline_dist_thr_ratio_ << std::endl;
    os << "\tRedundant obs ratio: " << settings.redundant_obs_ratio_thr_ << std::endl;
    os << "\tObserved ratio threshold: " << settings.observed_ratio_thr_ << std::endl;
    os << "\tNum obs keyframes threshold: " << settings.num_obs_keyfrms_thr_ << std::endl;

    os << "KeyframeInserter\n";
    os << "\tEnough lms threshold: " << settings.enough_lms_thr_ << std::endl;
    os << "\tMax interval: " << settings.max_interval_ << std::endl;
    os << "\tMin interval: " << settings.min_interval_ << std::endl;
    os << "\tMax distance: " << settings.max_distance_ << std::endl;
    os << "\tLms ratio thr almost all lms are tracked: " << settings.lms_ratio_thr_almost_all_lms_are_tracked_ << std::endl;
    os << "\tLms ratio thr views changed: " << settings.lms_ratio_thr_view_changed_ << std::endl;

    os << "Initializer\n";
    os << "\tUse fixed seed: " << settings.use_fixed_seed_ << std::endl;
    os << "\tNum ransac iterations: " << settings.num_ransac_iterations_ << std::endl;
    os << "\tMin num valid pts: " << settings.min_num_valid_pts_ << std::endl;
    os << "\tMin num triangulated pts: " << settings.min_num_triangulated_pts_ << std::endl;
    os << "\tNum ba iterations: " << settings.num_ba_iterations_ << std::endl;
    os << "\tParallax deg threshold: " << settings.parallax_deg_threshold_ << std::endl;
    os << "\tReprojection error threshold: " << settings.reprojection_error_threshold_ << std::endl;
    os << "\tScaling factor: " << settings.scaling_factor_ << std::endl;
    os << "\tOptimise focal length: " << settings.optimise_focal_length_ << std::endl;

    os << "LoopDetector\n";
    os << "\tLoop detector is enabled: " << settings.loop_detector_is_enabled_ << std::endl;
    os << "\tUse fixed seed: " << settings.loop_detector_use_fixed_seed_ << std::endl;
    os << "\tReject by graph distance: " << settings.reject_by_graph_distance_ << std::endl;
    os << "\tNum final matches threshold: " << settings.num_final_matches_threshold_ << std::endl;
    os << "\tMin continuity: " << settings.min_continuity_ << std::endl;    
    os << "\tMin distance on graph: " << settings.min_distance_on_graph_ << std::endl;
    os << "\tNum matches threshold: " << settings.num_matches_thr_ << std::endl;
    os << "\tNum matches threshold for robust matcher: " << settings.num_matches_thr_robust_matcher_ << std::endl;
    os << "\tNum optimized inliers threshold: " << settings.num_optimized_inliers_thr_ << std::endl;
    os << "\tTop n covisibilities to search: " << settings.top_n_covisibilities_to_search_ << std::endl;

    os << "Relocalizer\n";
    os << "\tUse fixed seed: " << settings.relocalizer_use_fixed_seed_ << std::endl;
    os << "\tMin num bow matches: " << settings.min_num_bow_matches_ << std::endl;
    os << "\tMin num valid obs: " << settings.min_num_valid_obs_ << std::endl;
    os << "\tBow match lowe ratio: " << settings.bow_match_lowe_ratio_ << std::endl;
    os << "\tProj match lowe ratio: " << settings.proj_match_lowe_ratio_ << std::endl;
    os << "\tRobust match lowe ratio: " << settings.robust_match_lowe_ratio_ << std::endl;

    os << "StereoRectifier\n";
    os << "\tK_left: "; for (auto el : settings.K_left_) os << el << " "; os << std::endl;
    os << "\tK_right: "; for (auto el : settings.K_right_) os << el << " "; os << std::endl;
    os << "\tR_left: "; for (auto el : settings.R_left_) os << el << " "; os << std::endl;
    os << "\tR_right: "; for (auto el : settings.R_right_) os << el << " "; os << std::endl;
    os << "\tD_left: "; for (auto el : settings.D_left_) os << el << " "; os << std::endl;
    os << "\tD_right: "; for (auto el : settings.D_right_) os << el << " "; os << std::endl;

    os << "System\n";
    os << "\tMap format: " << stella_vslam::io::map_format_to_string[static_cast<unsigned>(settings.map_format_)]
       << std::endl;
    return os;
}

void set_module_logger(std::shared_ptr<spdlog::logger> logger) {
    spdlog::set_default_logger(logger);
}

void set_module_log_level(spdlog::level::level_enum log_level) {
    spdlog::set_level(log_level);
}

} // namespace stella_vslam_bfx