/** \file
 * \brief Definition of focal length initialization from a fundamental matrix 
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <stella_vslam/type.h>

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

} // namespace stella_vslam_bfx
