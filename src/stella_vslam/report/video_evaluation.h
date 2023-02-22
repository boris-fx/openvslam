/** \file
 * \brief Definition of exported video evaluation classes 
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

//#include "stella_vslam/data/map_database.h"
#include <string>
#include <map>

#include "stella_vslam/exports.h"

#include <Eigen/Core>

namespace stella_vslam::data { class map_database; }
namespace stella_vslam::camera { class base; }

namespace stella_vslam_bfx {

class solve;

STELLA_VSLAM_API bool create_evaluation_video(std::string const& trackedVideoName, std::string const& testName,
                            stella_vslam::data::map_database const* map_db, std::map<double, int> const& timestampToVideoFrame,
                            std::map<int, Eigen::Matrix4d> const* videoFrameToCamera);

STELLA_VSLAM_API bool create_evaluation_video(std::string const& trackedVideoName, std::string const& testName,
                                              stella_vslam_bfx::solve const& final_solve);

// NB: See also (from Mocha/Cam3D/VideoEvaluation.h), which could be here
// 
// bool createEvaluationSequence(std::string_view const& outputBaseName,
//                               std::function<bool(int, cv::Mat&)> get_rgb_frame,
//                               stella_vslam_bfx::solve const& solve);
// 

} // namespace stella_vslam_bfx



