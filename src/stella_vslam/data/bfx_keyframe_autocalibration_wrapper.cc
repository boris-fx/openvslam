#include "stella_vslam/data/bfx_keyframe_autocalibration_wrapper.h"

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

bool keyframe_autocalibration_wrapper::operator()() const {
    return fx && fy && autocalibration_params && camera;
}

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
    }

    return true;
}

keyframe_autocalibration_wrapper::keyframe_autocalibration_wrapper(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms)
{
   using namespace stella_vslam;

    // Get the shared camera from a keyframe
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

    if (!camera)
        return;

    autocalibration_params = &camera->autocalibration_parameters_;

    //switch (camera->model_type_) {
    //    case camera::model_type_t::Perspective: {
    //        auto c = static_cast<camera::perspective*>(camera);
    //        fx = &c->fx_;
    //        fy = &c->fy_;
    //        break;
    //    }
    //    case camera::model_type_t::Fisheye: {
    //        auto c = static_cast<camera::fisheye*>(camera);
    //        fx = &c->fx_;
    //        fy = &c->fy_;
    //        break;
    //    }
    //    case camera::model_type_t::RadialDivision: {
    //        auto c = static_cast<camera::radial_division*>(camera);
    //        fx = &c->fx_;
    //        fy = &c->fy_;
    //        break;
    //    }
    //}
    fx_fy_from_camera(camera, &fx, &fy);
}

bool setFocalLengthXPixels(stella_vslam::data::map_database* map_db,
                           double focal_length_x_pixels)
{
    if (!map_db)
        return false;
    auto keyfrms = map_db->get_all_keyframes();

    keyframe_autocalibration_wrapper wrapper(keyfrms);
    if (!wrapper()) {
        spdlog::error("setFocalLengthXPixels failed!!!!!!!!!!!!!");
        return false;
    }
    //spdlog::error("setFocalLengthXPixels set to {}", focal_length_x_pixels);

    double par = *wrapper.fy / *wrapper.fx;
    *wrapper.fx = focal_length_x_pixels;
    *wrapper.fy = focal_length_x_pixels * par;

    return true;
}

bool setFocalLengthXPixels(stella_vslam::data::frame& frm, double focal_length_x_pixels)
{
    double *fx(nullptr), *fy(nullptr);
    bool ok = fx_fy_from_camera(frm.camera_, &fx, &fy);

    if (!ok) {
        spdlog::error("setFocalLengthXPixels failed!!!!!!!!!!!!!");
        return false;
    }
    //spdlog::error("setFocalLengthXPixels set to {}", focal_length_x_pixels);

    double par = *fy / *fx;
    *fx = focal_length_x_pixels;
    *fy = focal_length_x_pixels * par;

    return true;

}

} // namespace stella_vslam_bfx
