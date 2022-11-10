/** \file
 * \brief Definition of camera intrinsic parameters
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include "stella_vslam/exports.h"

#include <nlohmann/json_fwd.hpp>

namespace stella_vslam {
namespace data {

class frame;

class keyframe;

class map_database;

class STELLA_VSLAM_API bfx_shared_camera_intrinsics {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double focal_length_x_pixels;

    //! encode landmark information as JSON
    nlohmann::json to_json() const;

};

} // namespace data
} // namespace stella_vslam
