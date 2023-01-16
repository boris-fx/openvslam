#ifndef STELLA_VSLAM_BFX_CONFIG_SETTINGS_H
#define STELLA_VSLAM_BFX_CONFIG_SETTINGS_H

#include "stella_vslam/exports.h"
#include <ostream>
#include <array>
#include <vector>
#include <functional>

namespace stella_vslam::data { class map_database; }

// Moved from camera/base.h to avoid circular headers
namespace stella_vslam {
namespace camera {

enum class setup_type_t {
    Monocular = 0,
    Stereo = 1,
    RGBD = 2
};
const std::array<std::string, 3> setup_type_to_string = {{"Monocular", "Stereo", "RGBD"}};

enum class model_type_t {
    Perspective = 0,
    Fisheye = 1,
    Equirectangular = 2,
    RadialDivision = 3
};
const std::array<std::string, 4> model_type_to_string = {{"Perspective", "Fisheye", "Equirectangular", "RadialDivision"}};

enum class color_order_t {
    Gray = 0,
    RGB = 1,
    BGR = 2
};
const std::array<std::string, 3> color_order_to_string = {{"Gray", "RGB", "BGR"}};

} // namespace camera

// NB stella_vslam doesn't actually use the enums below; they
// create the appropriate object based on the YAML string.
// They may add enums in future; inventing ones now for consistency.
namespace marker_model {

    enum class model_type_t {
        Aruco = 0
    };
    const std::array<std::string, 1> model_type_to_string = {{"Aruco"}};

} // namespace marker_model

namespace io {

    enum class map_format_t {
        SQLite3 = 0,
        Msgpack = 1
    };
    const std::array<std::string, 2> map_format_to_string = {{"SQLite3", "msgpack"}};

} // namespace io

} // namespace stella_vslam

namespace stella_vslam_bfx {

class STELLA_VSLAM_API config_settings {
public:

    //! Constructor for perspective camera
    config_settings(stella_vslam::camera::model_type_t camera_model,
                    stella_vslam::camera::setup_type_t camera_setup,
                    stella_vslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps, double fx, double fy,
                    double cx, double cy, double p1, double p2,
                    double k1, double k2, double k3);
    
    //! Constructor for fisheye camera
    config_settings(stella_vslam::camera::model_type_t camera_model,
                    stella_vslam::camera::setup_type_t camera_setup,
                    stella_vslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps,
                    double fx, double fy, double cx, double cy,
                    double k1, double k2, double k3, double k4);
    
    //! Constructor for radial division camera
    config_settings(stella_vslam::camera::model_type_t camera_model,
                    stella_vslam::camera::setup_type_t camera_setup,
                    stella_vslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps,
                    double fx, double fy, double cx, double cy,
                    double distortion);
    
    //! Constructor for equirectangular camera
    config_settings(stella_vslam::camera::model_type_t camera_model,
                    stella_vslam::camera::setup_type_t camera_setup,
                    stella_vslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps);


    ~config_settings() {}

    friend std::ostream& operator<<(std::ostream& os, const config_settings& settings);

public:

    // Camera/image settings (common)
    stella_vslam::camera::model_type_t camera_model_;
    stella_vslam::camera::setup_type_t camera_setup_;
    stella_vslam::camera::color_order_t colour_order_;
    int cols_;
    int rows_;
    double fps_;

    // Settings for specific cameras
    // NB Equirectangular has no extra parameters
    union {
        struct {
            double fx_, fy_, cx_, cy_, p1_, p2_,
                    k1_, k2_, k3_;
        } perspective_settings_;

        struct {
            double fx_, fy_, cx_, cy_,
                    k1_, k2_, k3_, k4_;
        } fisheye_settings_;

        struct {
            double fx_, fy_, cx_, cy_;
            double distortion_;
        } radial_division_settings_;
    };

    // Stereo settings
    double focal_x_baseline_ = 0.0;
    double depth_threshold_ = 40.0;

    // Preprocessing parameters
    unsigned max_num_keypoints_ = 2000;
    unsigned ini_max_num_keypoints_ = 0;
    double depthmap_factor_ = 1.0;
    std::vector<std::vector<float>> mask_rectangles_;

    // ORB settings
    bool use_orb_features_ = true;
    bool undistort_prematches_ = true;
    unsigned ini_fast_threshold_ = 20;
    unsigned min_fast_threshold_ = 7;
    unsigned num_levels_ = 8;
    float scale_factor_ = 1.2f;

    // Marker parameters
    bool marker_model_is_enabled_ = false;
    stella_vslam::marker_model::model_type_t marker_model_ =
                stella_vslam::marker_model::model_type_t::Aruco;
    double marker_width_ = 0.0;
    double marker_size_ = 0.0;
    double max_markers_ = 0.0;

    // Tracking parameters
    bool enable_auto_relocalization_ = true;
    bool use_robust_matcher_for_relocalization_request_ = false;
    double reloc_distance_threshold_ = 0.2;
    double reloc_angle_threshold_ = 0.45;
    unsigned max_num_local_keyfrms_ = 60;

    // Mapping parameters
    bool use_baseline_dist_thr_ratio_ = false;
    bool use_additional_keyframes_for_monocular_ = false;
    unsigned num_obs_thr_ = 2;
    unsigned num_reliable_keyfrms_ = 2;
    unsigned desired_valid_obs_ = 0;
    unsigned num_obs_keyfrms_thr_ = 10;
    double baseline_dist_thr_ = 1.0;
    double baseline_dist_thr_ratio_ = 0.02;
    double redundant_obs_ratio_thr_ = 0.9;
    double observed_ratio_thr_ = 0.3;
    unsigned min_num_shared_lms_ = 15;
    unsigned top_n_covisibilities_to_search = 30;
    unsigned enable_interruption_of_landmark_generation_ = true;
    unsigned enable_interruption_before_local_BA_ = true;
    unsigned num_covisibilities_for_landmark_generation_ = 10;
    unsigned num_covisibilities_for_landmark_fusion_ = 10;

    // Keyframe settings
    unsigned enough_lms_thr_ = 100;
    double max_interval_ = 1.0;
    double min_interval_ = 0.1;
    double max_distance_ = -1.0;
    double lms_ratio_thr_almost_all_lms_are_tracked_ = 0.9;
    double lms_ratio_thr_view_changed_ = 0.8;

    // Solve/initializer settings
    bool use_fixed_seed_ = true;
    unsigned num_ransac_iterations_ = 100;
    unsigned min_num_valid_pts_ = 50;
    unsigned min_num_triangulated_pts_ = 50;
    unsigned num_ba_iterations_ = 20;
    float parallax_deg_threshold_ = 1.0f;
    float reprojection_error_threshold_ = 4.0f;
    float scaling_factor_ = 1.0f;
    bool optimise_focal_length_ = false;

    // Loop detector settings
    std::string optimizer_backend_ = "g2o";
    bool loop_detector_is_enabled_ = true;
    bool loop_detector_use_fixed_seed_ = true;
    bool reject_by_graph_distance_ = false;
    unsigned num_final_matches_threshold_ = 40;
    unsigned min_continuity_ = 3;
    unsigned min_distance_on_graph_ = 50;
    unsigned num_matches_thr_ = 20;
    unsigned num_matches_thr_robust_matcher_ = 0;
    unsigned num_optimized_inliers_thr_ = 20;
    unsigned top_n_covisibilities_to_search_ = 0;

    // Relocalizer settings
    bool relocalizer_use_fixed_seed_ = true;
    unsigned min_num_bow_matches_ = 20;
    unsigned min_num_valid_obs_ = 50;
    double bow_match_lowe_ratio_ = 0.75;
    double proj_match_lowe_ratio_ = 0.9;
    double robust_match_lowe_ratio_ = 0.8;

    // Stereo rectifier parameters TODO set defaults?
    std::vector<double> K_left_, K_right_, R_left_, R_right_, D_left_, D_right_;

    // System/IO
    std::string map_format_ = "msgpack";
};

} // namespace stella_vslam_bfx

#endif // STELLA_VSLAM_BFX_CONFIG_SETTINGS_H