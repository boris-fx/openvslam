#ifndef OPENVSLAM_CONFIG_SETTINGS_H
#define OPENVSLAM_CONFIG_SETTINGS_H

#include <ostream>
#include <array>
#include <vector>

// Moved from camera/base.h to avoid circular headers
namespace openvslam {
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
} // namespace openvslam

namespace openvslam_bfx {

class config_settings {
public:

    //! Constructor for perspective camera
    config_settings(openvslam::camera::model_type_t camera_type,
                    openvslam::camera::setup_type_t camera_setup,
                    openvslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps, double fx, double fy,
                    double cx, double cy, double p1, double p2,
                    double k1, double k2, double k3);
    
    //! Constructor for fisheye camera
    config_settings(openvslam::camera::model_type_t camera_type,
                    openvslam::camera::setup_type_t camera_setup,
                    openvslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps,
                    double fx, double fy, double cx, double cy,
                    double k1, double k2, double k3, double k4);
    
    //! Constructor for radial division camera
    config_settings(openvslam::camera::model_type_t camera_type,
                    openvslam::camera::setup_type_t camera_setup,
                    openvslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps,
                    double fx, double fy, double cx, double cy,
                    double distortion);
    
    //! Constructor for equirectangular camera
    config_settings(openvslam::camera::model_type_t camera_type,
                    openvslam::camera::setup_type_t camera_setup,
                    openvslam::camera::color_order_t colour_order,
                    int cols, int rows, double fps);


    ~config_settings() {}

    friend std::ostream& operator<<(std::ostream& os, const config_settings& settings);

public:

    // Camera/image settings (common)
    openvslam::camera::model_type_t camera_type_;
    openvslam::camera::setup_type_t camera_setup_;
    openvslam::camera::color_order_t colour_order_;
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
    unsigned ini_fast_threshold_ = 20;
    unsigned min_fast_threshold_ = 7;
    unsigned num_levels_ = 8;
    float orb_scale_factor_ = 1.2f;

    // Tracking parameters
    bool enable_auto_relocalization_ = true;
    bool use_robust_matcher_for_relocalization_request_ = false;
    double reloc_distance_threshold_ = 0.2;
    double reloc_angle_threshold_ = 0.45;

    // Mapping parameters
    bool use_baseline_dist_thr_ratio_ = false;
    double baseline_dist_thr_ = 1.0;
    double baseline_dist_thr_ratio_ = 0.02;
    double redundant_obs_ratio_thr_ = 0.9;

    // Keyframe settings
    double max_keyframe_interval_ = 1.0f;
    double lms_ratio_thr_almost_all_lms_are_tracked_ = 0.95;
    double lms_ratio_thr_view_changed_ = 0.9;

    // Solve/initializer settings
    bool solve_use_fixed_seed_ = true;
    unsigned num_ransac_iterations_ = 100;
    unsigned min_num_triangulated_ = 50;
    unsigned num_ba_iterations_ = 20;
    float parallax_deg_threshold_ = 1.0f;
    float reprojection_error_threshold_ = 4.0f;
    float scaling_factor_ = 1.0f;

    // Loop detector settings
    bool loop_detector_is_enabled_ = true;
    bool loop_detector_use_fixed_seed_ = true;
    unsigned num_final_matches_threshold_ = 40;
    unsigned min_continuity_ = 3;

    // Relocalizer settings
    bool relocalizer_use_fixed_seed_ = true;
    unsigned min_num_bow_matches_ = 20;
    unsigned min_num_valid_obs_ = 50;
    double bow_match_lowe_ratio_ = 0.75;
    double proj_match_lowe_ratio_ = 0.9;
    double robust_match_lowe_ratio_ = 0.8;

    // BoW settings
    bool reject_by_graph_distance_ = false;
    int loop_min_distance_on_graph_ = 30;

    // Stereo rectifier parameters TODO set defaults?
    std::vector<double> K_left_, K_right_, R_left_, R_right_, D_left_, D_right_;
};

} // namespace openvslam_bfx

#endif // OPENVSLAM_CONFIG_SETTINGS_H