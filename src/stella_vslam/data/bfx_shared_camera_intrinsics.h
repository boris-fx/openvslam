/** \file
 * \brief Definition of camera intrinsic parameters
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include "stella_vslam/exports.h"

#include <nlohmann/json_fwd.hpp>

namespace stella_vslam::data {
class frame;
class keyframe;
class map_database;
} // namespace stella_vslam::data

namespace stella_vslam_bfx {

struct autocalibration_parameters;

/** Populate the shared keyframe camera from a camera intrinsics vertex **/
struct keyframe_autocalibration_wrapper {
    double* fx = nullptr;
    double* fy = nullptr;
    stella_vslam_bfx::autocalibration_parameters* autocalibration_params = nullptr;

    //! \return true if none of the pointers is nullptr
    bool operator()() const;

    keyframe_autocalibration_wrapper(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms);
};

bool setFocalLengthXPixels(stella_vslam::data::map_database * map_db, double focal_length_x_pixels);

bool setFocalLengthXPixels(stella_vslam::data::frame& frm, double focal_length_x_pixels);

} // namespace stella_vslam_bfx

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
