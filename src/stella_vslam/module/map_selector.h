/** \file
 * \brief Definition of the interface between the stella-vslam based solver and apps using it (Mocha, run_video_slam, etc...)
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <list>

namespace stella_vslam::data { class map_database; }
namespace stella_vslam { enum class tracker_state_t; }

namespace stella_vslam_bfx {

/**
 * \brief Logic for deciding when to abandon a map, and for storing and retrieving abandoned maps
 */
struct map_selector {
    bool enabled = false; // In use or not
    bool allow_reset = true;
    int track_fail_count = 0;
    //! Call when tracking fails to register the failure. Returns true if the map should be reset.
    bool should_reset_map_for_tracking_failure(stella_vslam::data::map_database const* map_db);
    void map_reset(stella_vslam::data::map_database* map_db);

    void restore_best_map(stella_vslam::data::map_database& map_db, stella_vslam::tracker_state_t & tracking_state);

    //
    std::list<stella_vslam::data::map_database> stored_maps;
    //data::map_database stored_maps;
};




} // namespace stella_vslam_bfx
