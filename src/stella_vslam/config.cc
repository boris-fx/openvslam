#include "stella_vslam/config.h"
#include "stella_vslam/marker_model/aruco.h"
#include "stella_vslam/util/string.h"

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

namespace stella_vslam {

config::config(const stella_vslam_bfx::config_settings& settings)
    : settings_(settings) {
    spdlog::debug("CONSTRUCT: config");

    //========================//
    // Load Marker Parameters //
    //========================//

    if (settings.marker_model_is_enabled_) {
        spdlog::debug("load marker model parameters");
        if (settings.marker_model_ == stella_vslam::marker_model::model_type_t::Aruco) {
            marker_model_ = std::make_shared<marker_model::aruco>(
                settings_.marker_width_,
                settings_.marker_size_,
                settings_.max_markers_);
        }
        else {
            throw std::runtime_error("Invalid marker model type :" + 
                stella_vslam::marker_model::model_type_to_string[static_cast<unsigned>(settings_.marker_model_)]);
        }
    }
}

config::~config() {
    spdlog::debug("DESTRUCT: config");
}

std::ostream& operator<<(std::ostream& os, const config& cfg) {
    os << cfg.settings_;
    return os;
}

} // namespace stella_vslam
