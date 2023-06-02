#include "map_selector.h"

#include <spdlog/spdlog.h>

#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/map_database.h>

#include <stella_vslam/report/metrics.h>

#include <stella_vslam/tracking_module.h>

using namespace stella_vslam;

namespace stella_vslam_bfx {

bool map_selector::should_reset_map_for_tracking_failure(data::map_database const* map_db) {
    if (!enabled)
        return false;
    if (!allow_reset)
        return false;
    ++track_fail_count;
    int keyframe_count = map_db->get_num_keyframes();

#if 0 // Allow some fails (based on map size) before resetting
    if (keyframe_count < 5)
        return track_fail_count > 0; // Allow no fails for less than 5 keyframes
    if (keyframe_count < 10)
        return track_fail_count > 1; // Allow 1 fail for 5-9 keyframes
    if (keyframe_count < 15)
        return track_fail_count > 2; // Allow 2 fails for 10-14 keyframes
    if (keyframe_count < 25)
        return track_fail_count > 2; // Allow 3 fails for 15-24 keyframes
    return false; // Allow unlimited fails for more than 25 keyframes
#endif

    return true; // Always reset on first failure, when resetting is enabled
}

std::optional<metrics::candidate_map_stats> map_info(data::map_database const* map_db)
{
    if (!map_db)
        return std::nullopt;
    auto keyframes = map_db->get_all_keyframes();
    std::set<int> frames;
    for (auto const& keyframe : keyframes) {
        std::optional<stage_and_frame> stage_and_frame = stella_vslam_bfx::metrics::get_instance()->timestamp_to_frame(keyframe->timestamp_);
        if (stage_and_frame)
            frames.insert(stage_and_frame.value().frame);
    }
    if (frames.empty())
        return std::nullopt;
    return metrics::candidate_map_stats({ { *frames.begin(), *frames.rbegin() }, 12 });
}

void map_selector::map_reset(data::map_database* map_db)
{
    stored_maps.push_back(*map_db);

    // Add the map frame range to the metrics data
    if (auto candidate_map_info = map_info(map_db))
        stella_vslam_bfx::metrics::get_instance()->candidate_map_list.push_back(candidate_map_info.value());

    spdlog::info("map stored. Num stored maps {}", stored_maps.size());

    track_fail_count = 0;
}

void map_selector::restore_best_map(data::map_database& map_db, tracker_state_t &tracking_state)
{
    // Add the final map frame range to the metrics data
    if (auto candidate_map_info = map_info(&map_db))
        stella_vslam_bfx::metrics::get_instance()->candidate_map_list.push_back(candidate_map_info.value());

    spdlog::info("Restoring best map. Current map size {} Num stored maps {}", map_db.get_num_keyframes(), stored_maps.size());

    data::map_database* best_map_db = &map_db;
    unsigned int best_map_keyframe_count = map_db.get_num_keyframes();
    for (auto& map : stored_maps) {
        unsigned int keyframe_count = map.get_num_keyframes();
        spdlog::info("Found map of size {}", keyframe_count);

        if (best_map_keyframe_count < keyframe_count) {
            best_map_db = &map;
            best_map_keyframe_count = keyframe_count;
        }
    }

    if (best_map_db != &map_db) {
        spdlog::info("Replacing map of size {}, with stored map of size {}", map_db.get_num_keyframes(), best_map_keyframe_count);
        map_db = *best_map_db;
        tracking_state = tracker_state_t::Tracking;
    }

}

} // namespace stella_vslam_bfx