#include "stella_vslam/data/keyframe_autocalibration_wrapper.h"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "stella_vslam/camera/base.h"
#include "stella_vslam/camera/perspective.h"
#include "stella_vslam/camera/fisheye.h"
#include "stella_vslam/camera/radial_division.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/map_database.h"

namespace stella_vslam_bfx {

//bool keyframe_autocalibration_wrapper::operator()() const {
//    return fx && fy && autocalibration_params && camera;
//}

bool fx_fy_from_camera(stella_vslam::camera::base* camera, double **fx, double **fy)
{
   using namespace stella_vslam;
    if (!camera)
        return false;

    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera);
            *fx = &c->fx_;
            *fy = &c->fy_;
            break;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            *fx = &c->fx_;
            *fy = &c->fy_;
            break;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division*>(camera);
            *fx = &c->fx_;
            *fy = &c->fy_;
            break;
        }
        default:
            return false;
    }

    return true;
}

template<typename camera_type>
void intrinsics_from_camera(camera_type* camera, double& focal_length_x_pixels, double& par, double& cx, double& cy)
{
    focal_length_x_pixels = camera->fx_;
    par = camera->fy_ / camera->fx_;
    cx = camera->cx_;
    cy = camera->cy_;
}

bool intrinsics_from_camera(stella_vslam::camera::base const* camera, double &focal_length_x_pixels, double &par, double &cx, double &cy)
{
    using namespace stella_vslam;
    if (!camera)
        return false;

    switch (camera->model_type_) {
    case camera::model_type_t::Perspective: {
        intrinsics_from_camera(static_cast<camera::perspective const*>(camera), focal_length_x_pixels, par, cx, cy);
        break;
    }
    case camera::model_type_t::Fisheye: {
        intrinsics_from_camera(static_cast<camera::fisheye const*>(camera), focal_length_x_pixels, par, cx, cy);
        break;
    }
    case camera::model_type_t::RadialDivision: {
        intrinsics_from_camera(static_cast<camera::radial_division const*>(camera), focal_length_x_pixels, par, cx, cy);
        break;
    }
    default:
        return false;
    }

    return true;
}

stella_vslam::camera::base* camera_from_keyframes(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms)
{
    // Get the shared camera from a keyframe
    stella_vslam::camera::base* camera(nullptr);
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }
        camera = keyfrm->camera_;
        break;
    }

    return camera;
}

//keyframe_autocalibration_wrapper::keyframe_autocalibration_wrapper(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms)
//{
//   using namespace stella_vslam;
//
//    // Get the shared camera from a keyframe
//    for (const auto& keyfrm : keyfrms) {
//        if (!keyfrm) {
//            continue;
//        }
//        if (keyfrm->will_be_erased()) {
//            continue;
//        }
//        camera = keyfrm->camera_;
//        break;
//    }
//
//    if (!camera)
//        return;
//
//    autocalibration_params = &camera->autocalibration_parameters_;
//
//    //switch (camera->model_type_) {
//    //    case camera::model_type_t::Perspective: {
//    //        auto c = static_cast<camera::perspective*>(camera);
//    //        fx = &c->fx_;
//    //        fy = &c->fy_;
//    //        break;
//    //    }
//    //    case camera::model_type_t::Fisheye: {
//    //        auto c = static_cast<camera::fisheye*>(camera);
//    //        fx = &c->fx_;
//    //        fy = &c->fy_;
//    //        break;
//    //    }
//    //    case camera::model_type_t::RadialDivision: {
//    //        auto c = static_cast<camera::radial_division*>(camera);
//    //        fx = &c->fx_;
//    //        fy = &c->fy_;
//    //        break;
//    //    }
//    //}
//    fx_fy_from_camera(camera, &fx, &fy);
//}

double focal_length_x_pixels_from_camera(stella_vslam::camera::base const* camera)
{
    double *fx(nullptr), *fy(nullptr);
    bool ok = fx_fy_from_camera(const_cast<stella_vslam::camera::base*>(camera), &fx, &fy);
    if (ok && fx)
        return *fx;
    return 0;
}

bool set_camera_focal_length_x_pixels(stella_vslam::camera::base* camera, double focal_length_x_pixels)
{
    using namespace stella_vslam;

    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera);
            camera::perspective ref(*c);
            double par = ref.fy_ / ref.fx_;
            double fx = focal_length_x_pixels;
            double fy = focal_length_x_pixels * par;
            camera::perspective alt(ref.name_, ref.setup_type_, ref.color_order_,
                                    ref.autocalibration_parameters_,
                                    ref.cols_, ref.rows_, ref.fps_,
                                    fx, fy, ref.cx_, ref.cy_,
                                    ref.k1_, ref.k2_, ref.p1_, ref.p2_, ref.k3_,
                                    ref.focal_x_baseline_, ref.depth_thr_);
            // Copy the bits that are affected by focal length. NB: camera::perspective's copy and move seem to be deleted
            const_cast<double&>(c->fx_) = alt.fx_;
            const_cast<double&>(c->fy_) = alt.fy_;
            const_cast<double&>(c->fx_inv_) = alt.fx_inv_;
            const_cast<double&>(c->fy_inv_) = alt.fy_inv_;
            const_cast<cv::Mat&>(c->cv_cam_matrix_) = alt.cv_cam_matrix_;
            const_cast<Mat33_t&>(c->eigen_cam_matrix_) = alt.eigen_cam_matrix_;
            return true;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            camera::fisheye ref(*c);
            double par = ref.fy_ / ref.fx_;
            double fx = focal_length_x_pixels;
            double fy = focal_length_x_pixels * par;
            camera::fisheye alt(ref.name_, ref.setup_type_, ref.color_order_,
                                ref.autocalibration_parameters_,
                                ref.cols_, ref.rows_, ref.fps_,
                                fx, fy, ref.cx_, ref.cy_,
                                ref.k1_, ref.k2_, ref.k3_, ref.k4_,
                                ref.focal_x_baseline_, ref.depth_thr_);
            // Copy the bits that are affected by focal length. NB: camera::fisheye's copy and move seem to be deleted
            const_cast<double&>(c->fx_) = alt.fx_;
            const_cast<double&>(c->fy_) = alt.fy_;
            const_cast<double&>(c->fx_inv_) = alt.fx_inv_;
            const_cast<double&>(c->fy_inv_) = alt.fy_inv_;
            const_cast<cv::Mat&>(c->cv_cam_matrix_) = alt.cv_cam_matrix_;
            const_cast<Mat33_t&>(c->eigen_cam_matrix_) = alt.eigen_cam_matrix_;
            return true;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division*>(camera);
            camera::radial_division ref(*c);
            double par = ref.fy_ / ref.fx_;
            double fx = focal_length_x_pixels;
            double fy = focal_length_x_pixels * par;
            camera::radial_division alt(ref.name_, ref.setup_type_, ref.color_order_,
                                        ref.autocalibration_parameters_,
                                        ref.cols_, ref.rows_, ref.fps_,
                                        fx, fy, ref.cx_, ref.cy_,
                                        ref.distortion_, ref.focal_x_baseline_, ref.depth_thr_);
            // Copy the bits that are affected by focal length. NB: camera::radial_division's copy and move seem to be deleted
            const_cast<double&>(c->fx_) = alt.fx_;
            const_cast<double&>(c->fy_) = alt.fy_;
            const_cast<double&>(c->fx_inv_) = alt.fx_inv_;
            const_cast<double&>(c->fy_inv_) = alt.fy_inv_;
            const_cast<cv::Mat&>(c->cv_cam_matrix_) = alt.cv_cam_matrix_;
            const_cast<Mat33_t&>(c->eigen_cam_matrix_) = alt.eigen_cam_matrix_;
            return true;
        }
    }
    return false;
}

//bool set_camera_focal_length_x_pixels(stella_vslam::data::map_database* map_db,
//                           double focal_length_x_pixels)
//{
//    if (!map_db)
//        return false;
//    auto keyfrms = map_db->get_all_keyframes();
//
//    stella_vslam::camera::base* camera = camera_from_keyframes(keyfrms);
//    if (!camera) {
//        spdlog::error("set_camera_focal_length_x_pixels failed !");
//        return false;
//    }
//    return set_camera_focal_length_x_pixels(camera, focal_length_x_pixels);
//}
//
//bool set_camera_focal_length_x_pixels(stella_vslam::data::frame& frm, double focal_length_x_pixels)
//{
//    return set_camera_focal_length_x_pixels(frm.camera_, focal_length_x_pixels);
//}

} // namespace stella_vslam_bfx
