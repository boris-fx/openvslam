/** \file
 * \brief Definition of focal length initialization from a fundamental matrix 
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <stella_vslam/type.h>
#include <stella_vslam/exports.h>

namespace stella_vslam::camera { class base; }
   
namespace stella_vslam_bfx {

/**
* \returns true if the camera focal length was changed 
* 
* Focal length not calculated - returns \c false, focal_length_estimate_is_stable \c true
* Focal length calculated but bad  - returns \c false, focal_length_estimate_is_stable \c false - block initialisation
* Focal length calculated but good - returns \c true, focal_length_estimate_is_stable \c true
* 
**/
bool initialize_focal_length(stella_vslam::Mat33_t const& F_21, stella_vslam::camera::base* camera, bool *focal_length_estimate_is_stable);

double min_geometric_error_focal_length(stella_vslam::Mat33_t const& F_21, stella_vslam::camera::base const* camera, bool* focal_length_estimate_is_stable);

std::set<double> candidateFocalLengthsOverFOVRange(double startDegrees, double endDegrees, double imageWidth);
double error_for_focal_length(stella_vslam::Mat33_t const& F_21, stella_vslam::camera::base const* camera, double focal_length_x_pixels);
double error_for_focal_length(stella_vslam::Mat33_t const& F_21, double focal_length_x_pixels, double par, double cx, double cy);

std::shared_ptr<stella_vslam::camera::base> modified_focal_length_camera_copy(stella_vslam::camera::base const* camera, double focal_length_x_pixels);

STELLA_VSLAM_API bool fundamental_to_focal_length_optimisation_test();

} // namespace stella_vslam_bfx
