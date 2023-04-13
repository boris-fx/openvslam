/** \file
 * \brief Definition of camera intrinsic parameters
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <optional>

#include "stella_vslam/exports.h"

#include <nlohmann/json_fwd.hpp>

namespace stella_vslam::data {
class keyframe;
} // namespace stella_vslam::data
namespace stella_vslam::camera {
class base;
} // namespace stella_vslam::camera

namespace stella_vslam_bfx {

// NB: stella creates the camera in system (based on cfg) and frames and keyframes have a pointer to it  
stella_vslam::camera::base* camera_from_keyframes(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms);

bool intrinsics_from_camera(stella_vslam::camera::base const* camera, double& focal_length_x_pixels, double& par, double& cx, double& cy);

double focal_length_x_pixels_from_camera(stella_vslam::camera::base const* camera);

bool set_camera_focal_length_x_pixels(stella_vslam::camera::base* camera, double focal_length_x_pixels);

} // namespace stella_vslam_bfx

