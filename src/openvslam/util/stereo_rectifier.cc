#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/util/stereo_rectifier.h"

#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>

namespace openvslam {
namespace util {

stereo_rectifier::stereo_rectifier(const std::shared_ptr<openvslam::config>& cfg)
    : stereo_rectifier(cfg->camera_, cfg->settings_) {}

stereo_rectifier::stereo_rectifier(camera::base* camera, const openvslam_bfx::config_settings& settings)
    : model_type_(load_model_type(settings)) {
    spdlog::debug("CONSTRUCT: util::stereo_rectifier");
    if (camera->setup_type_ != camera::setup_type_t::Stereo) {
        throw std::runtime_error("When stereo rectification is used, 'setup' must be set to 'stereo'");
    }
    if (camera->model_type_ != camera::model_type_t::Perspective) {
        throw std::runtime_error("When stereo rectification is used, 'model' must be set to 'perspective'");
    }
    // set image size
    const cv::Size img_size(camera->cols_, camera->rows_);
    // set camera matrices
    const auto K_l = parse_vector_as_mat(cv::Size(3, 3), settings.K_left_);
    const auto K_r = parse_vector_as_mat(cv::Size(3, 3), settings.K_right_);
    // set rotation matrices
    const auto R_l = parse_vector_as_mat(cv::Size(3, 3), settings.R_left_);
    const auto R_r = parse_vector_as_mat(cv::Size(3, 3), settings.R_right_);
    // set distortion parameters depending on the camera model
    const auto D_l_vec = settings.D_left_;
    const auto D_r_vec = settings.D_right_;
    const auto D_l = parse_vector_as_mat(cv::Size(1, D_l_vec.size()), D_l_vec);
    const auto D_r = parse_vector_as_mat(cv::Size(1, D_r_vec.size()), D_r_vec);
    // get camera matrix after rectification
    const auto K_rect = static_cast<camera::perspective*>(camera)->cv_cam_matrix_;
    // create undistortion maps
    switch (model_type_) {
        case camera::model_type_t::Perspective: {
            cv::initUndistortRectifyMap(K_l, D_l, R_l, K_rect, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
            cv::initUndistortRectifyMap(K_r, D_r, R_r, K_rect, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
            break;
        }
        case camera::model_type_t::Fisheye: {
            cv::fisheye::initUndistortRectifyMap(K_l, D_l, R_l, K_rect, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
            cv::fisheye::initUndistortRectifyMap(K_r, D_r, R_r, K_rect, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
            break;
        }
        default: {
            throw std::runtime_error("Invalid model type for stereo rectification: " + camera->get_model_type_string());
        }
    }
}

stereo_rectifier::~stereo_rectifier() {
    spdlog::debug("DESTRUCT: util::stereo_rectifier");
}

void stereo_rectifier::rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                               cv::Mat& out_img_l, cv::Mat& out_img_r) const {
    cv::remap(in_img_l, out_img_l, undist_map_x_l_, undist_map_y_l_, cv::INTER_LINEAR);
    cv::remap(in_img_r, out_img_r, undist_map_x_r_, undist_map_y_r_, cv::INTER_LINEAR);
}

cv::Mat stereo_rectifier::parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec) {
    if ( vec.size() != shape.area() )
        throw std::runtime_error("Wrong number of matrix elements for stereo rectification matrix");
    
    cv::Mat mat(shape, CV_64F);
    std::memcpy(mat.data, vec.data(), shape.height * shape.width * sizeof(double));
    return mat;
}

camera::model_type_t stereo_rectifier::load_model_type(const openvslam_bfx::config_settings& settings) {
    return settings.camera_type_;
}

} // namespace util
} // namespace openvslam
