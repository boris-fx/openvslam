#include "openvslam/config.h"
#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/camera/radial_division.h"
#include "openvslam/util/string.h"

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

namespace openvslam {

config::config(const openvslam_bfx::config_settings& settings)
    : settings_(settings) {
    spdlog::debug("CONSTRUCT: config");

    //========================//
    // Load Camera Parameters //
    //========================//

    spdlog::debug("load camera model type");
    const auto camera_model_type = camera::base::load_model_type(settings_);

    spdlog::debug("load camera model parameters");
    try {
        switch (camera_model_type) {
            case camera::model_type_t::Perspective: {
                camera_ = new camera::perspective(settings_);
                break;
            }
            case camera::model_type_t::Fisheye: {
                camera_ = new camera::fisheye(settings_);
                break;
            }
            case camera::model_type_t::Equirectangular: {
                camera_ = new camera::equirectangular(settings_);
                break;
            }
            case camera::model_type_t::RadialDivision: {
                camera_ = new camera::radial_division(settings_);
                break;
            }
        }
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading camera model parameters: {}", e.what());
        if (camera_) {
            delete camera_;
            camera_ = nullptr;
        }
        throw;
    }

    if (camera_->setup_type_ == camera::setup_type_t::Stereo || camera_->setup_type_ == camera::setup_type_t::RGBD) {
        if (camera_->model_type_ == camera::model_type_t::Equirectangular) {
            throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
        }
    }

    spdlog::debug("load ORB parameters");
    try {
        orb_params_ = new feature::orb_params(settings_);
    }
    catch (const std::exception& e) {
        spdlog::debug("failed in loading ORB feature extraction model: {}", e.what());
        if (orb_params_) {
            delete orb_params_;
            orb_params_ = nullptr;
        }
        throw;
    }
}

config::~config() {
    delete camera_;
    camera_ = nullptr;

    delete orb_params_;
    orb_params_ = nullptr;

    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.settings_;
    return os;
}

} // namespace openvslam
