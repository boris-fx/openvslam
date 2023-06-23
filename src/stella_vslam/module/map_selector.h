/** \file
 * \brief Definition of the interface between the stella-vslam based solver and apps using it (Mocha, run_video_slam, etc...)
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <list>
#include <vector>
#include <map>
#include <memory>
#include <string>

namespace stella_vslam::data { class map_database; }
namespace stella_vslam { enum class tracker_state_t; class system;  }

#define STORE_BINARY_MAPS 1

namespace stella_vslam_bfx {

/**
 * \brief Logic for deciding when to abandon a map, and for storing and retrieving abandoned maps
 */
class map_selector {
public:
    //inline static const std::string test_file = "map_selector";
    inline static const std::string test_file = "";

    bool enabled = false; // In use or not
    bool allow_reset = true;
    int track_fail_count = 0;
    //! Call when tracking fails to register the failure. Returns true if the map should be reset.
    bool should_reset_map_for_tracking_failure(stella_vslam::data::map_database const* map_db);
    
    void store_abandoned_map(stella_vslam::data::map_database* map_db);

    //bool store_final_map_metrics();
    //bool restore_best_map(stella_vslam::data::map_database& map_db);
    //bool have_better_map(stella_vslam::data::map_database& map_db);
    
    // Get the best map and store metrics
    std::shared_ptr<std::vector<unsigned char>> get_best_map(stella_vslam::data::map_database& map_db);

    void set_system(std::shared_ptr<stella_vslam::system> slam);

protected:
#if STORE_BINARY_MAPS
    struct stored_binary_map { std::shared_ptr<std::vector<unsigned char>> binary_map; std::pair<int, int> frame_range; int key_count; int fail_count;  };
    std::multimap<unsigned int, stored_binary_map> stored_binary_maps; // Each map is number of keyframes, map data
    
#else
    std::list<stella_vslam::data::map_database> stored_maps;
#endif
    std::shared_ptr<stella_vslam::system> slam_;
};




} // namespace stella_vslam_bfx
