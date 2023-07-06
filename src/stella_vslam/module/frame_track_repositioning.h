/** \file
 * \brief Definition of the interface between the stella-vslam based solver and apps using it (Mocha, run_video_slam, etc...)
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <stella_vslam/report/initialisation_debugging.h> // for stage_and_frame

namespace stella_vslam { class system; }

namespace stella_vslam_bfx {


    bool reposition_tracking_to_first_map_keyframe(std::shared_ptr<stella_vslam::system> slam,
                                                   std::map<double, stage_and_frame> const& timestamp_to_stage_and_frame,
                                                   int& reposition_frame);


} // namespace stella_vslam_bfx


