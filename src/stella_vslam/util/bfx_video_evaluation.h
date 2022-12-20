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

namespace stella_vslam::data { class map_database; }

namespace stella_vslam_bfx {

STELLA_VSLAM_API bool bfx_create_evaluation_video(std::string const& trackedVideoName, std::string const& testName, stella_vslam::data::map_database const* map_db, std::map<double, int> const& timestampToVideoFrame);


} // namespace stella_vslam_bfx



