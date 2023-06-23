#include "map_selector.h"

#include <spdlog/spdlog.h>

#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/map_database.h>

#include <stella_vslam/report/metrics.h>

#include <stella_vslam/tracking_module.h>
#include <stella_vslam/system.h>

using namespace stella_vslam;

namespace stella_vslam_bfx {

bool map_selector::should_reset_map_for_tracking_failure(data::map_database const* map_db) {
    spdlog::info("[] map_selector::should_reset_map_for_tracking_failure - start, now {} {}", track_fail_count, fmt::ptr(map_db));
    if (!enabled)
        return false;
    if (!allow_reset)
        return false;
    ++track_fail_count;
    spdlog::info("[] map_selector::should_reset_map_for_tracking_failure - track_fail_count incremented, now {}", track_fail_count);
    int keyframe_count = map_db->get_num_keyframes();

#if 1 // Allow some fails (based on map size) before resetting
    if (keyframe_count < 5)
        return track_fail_count > 0; // Allow no fails for less than 5 keyframes
    if (keyframe_count < 10)
        return track_fail_count > 1; // Allow 1 fail for 5-9 keyframes
    if (keyframe_count < 15)
        return track_fail_count > 2; // Allow 2 fails for 10-14 keyframes
    if (keyframe_count < 25)
        return track_fail_count > 3; // Allow 3 fails for 15-24 keyframes
    return false; // Allow unlimited fails for more than 25 keyframes
#else
    //if (keyframe_count > 20 && track_fail_count < 2) // Keep if more than 20 keyframes and less than 2 fails
    //if (keyframe_count > 0 && track_fail_count < 0) // Never keep (on fail)
    if (keyframe_count > 0 && track_fail_count < 2) // Keep if less than 2 fails
        return false; // Carry on tracking without reset
#endif

    return true; // Always reset on first failure, when resetting is enabled
}

std::optional<metrics::candidate_map_stats> map_info(data::map_database const* map_db, bool abandoned, bool restored, int fail_count)
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
    return metrics::candidate_map_stats({ { *frames.begin(), *frames.rbegin() }, (int)keyframes.size(), fail_count, abandoned, restored });
}

void map_selector::store_abandoned_map(data::map_database* map_db)
{
#if STORE_BINARY_MAPS
    if (!test_file.empty()) {
        static int hit_count = 0;
        ++hit_count;
        auto filename = test_file + std::to_string(hit_count) + ".msg";
        bool save_was_ok = slam_->save_map_database(filename);
        if (save_was_ok)
            spdlog::info("Map stored to file: {}", filename);
        else
            spdlog::info("Failed to store map to file: {}", filename);
    }
    std::pair<unsigned int, stored_binary_map> stored_map(map_db->get_num_keyframes(), {});
    stored_map.second.binary_map = std::make_shared<std::vector<unsigned char>>();
    slam_->save_map_database_to_memory(*stored_map.second.binary_map);
    auto candidate_map_info = map_info(map_db, true, false, track_fail_count);
    stored_map.second.frame_range = candidate_map_info->frame_range;
    stored_map.second.key_count = candidate_map_info->key_count;
    stored_map.second.fail_count = candidate_map_info->fail_count;
    stored_binary_maps.insert(stored_map);
    unsigned int num_stored_maps = stored_binary_maps.size();


#else
    stored_maps.push_back(*map_db);
    unsigned int num_stored_maps = stored_maps.size();

    // Add the map frame range to the metrics data
    if (auto candidate_map_info = map_info(map_db, true))
        stella_vslam_bfx::metrics::get_instance()->candidate_map_list.push_back(candidate_map_info.value());
#endif



    spdlog::info("map stored. Num stored maps {}", num_stored_maps);

    spdlog::info("[] map_selector::store_abandoned_map - track_fail_count before reset {}", track_fail_count);
    track_fail_count = 0;
    spdlog::info("[] map_selector::store_abandoned_map - track_fail_count after reset {}", track_fail_count);
}

//bool map_selector::store_final_map_metrics()
//{
//    data::map_database& map_db = *slam_->map_db_;
//    if (auto candidate_map_info = map_info(&map_db, false))
//        stella_vslam_bfx::metrics::get_instance()->candidate_map_list.push_back(candidate_map_info.value());
//}


//bool map_selector::restore_best_map(data::map_database& map_db)
//{
//    //data::map_database& map_db = *slam_->map_db_;
//    //tracker_state_t& tracking_state = slam_->tracker_->tracking_state_;
//
//    //// Add the final map frame range to the metrics data
//    //if (auto candidate_map_info = map_info(&map_db, false))
//    //    stella_vslam_bfx::metrics::get_instance()->candidate_map_list.push_back(candidate_map_info.value());
//
//#if STORE_BINARY_MAPS
//    unsigned int num_stored_maps = stored_binary_maps.size();
//#else
//    unsigned int num_stored_maps = stored_maps.size();
//#endif
//    spdlog::info("Restoring best map. Current map size {} Num stored maps {}", map_db.get_num_keyframes(), num_stored_maps);
//
//#if STORE_BINARY_MAPS
//    if (!stored_binary_maps.empty()) {
//        auto const best_stored = stored_binary_maps.rbegin();
//        if (map_db.get_num_keyframes() < best_stored->first) {
//            spdlog::info("Replacing map of size {}, with stored map of size {}", map_db.get_num_keyframes(), best_stored->first);
//
//            //slam_->shutdown();
//            bool ok = slam_->load_map_database_from_memory(*best_stored->second.binary_map);
//            if (!ok)
//                return false;
//            //slam_->startup(false);
//
//            //tracking_state = tracker_state_t::Tracking;
//            //tracking_state = tracker_state_t::Lost;
//            slam_->tracker_->tracking_state_ = tracker_state_t::Tracking;
//            return true;
//        }
//    }
//#else
//    data::map_database* best_map_db = &map_db;
//    unsigned int best_map_keyframe_count = map_db.get_num_keyframes();
//    for (auto& map : stored_maps) {
//        unsigned int keyframe_count = map.get_num_keyframes();
//        spdlog::info("Found map of size {}", keyframe_count);
//
//        if (best_map_keyframe_count < keyframe_count) {
//            best_map_db = &map;
//            best_map_keyframe_count = keyframe_count;
//        }
//    }
//
//    if (best_map_db != &map_db) {
//        spdlog::info("Replacing map of size {}, with stored map of size {}", map_db.get_num_keyframes(), best_map_keyframe_count);
//
//        auto keys = best_map_db->get_all_keyframes();
//        for (auto const& key : keys)
//            spdlog::info("    Key id: {}", key->id_);
//
//        map_db = *best_map_db;
//        tracking_state = tracker_state_t::Tracking;
//
//        return true;
//    }
//#endif
//
//    return false;
//}

std::shared_ptr<std::vector<unsigned char>> map_selector::get_best_map(data::map_database& map_db)
{
#if STORE_BINARY_MAPS
    bool restoring_first_stored_map = !stored_binary_maps.empty() && stored_binary_maps.rbegin()->first > map_db.get_num_keyframes();

    // Set the metrics for current and abandoned maps
    std::map<int, metrics::candidate_map_stats> candidate_map;
    auto current_map_info = map_info(&map_db, false, !restoring_first_stored_map, 0);
    if (current_map_info)
        candidate_map[current_map_info->frame_range.first] = current_map_info.value();
    for (auto const& stored_map : stored_binary_maps) {
        metrics::candidate_map_stats stats;
        stats.abandoned = true;
        stats.frame_range = stored_map.second.frame_range;
        stats.key_count = stored_map.second.key_count;
        stats.fail_count = stored_map.second.fail_count;
        stats.selected = restoring_first_stored_map  && &stored_map.first == &stored_binary_maps.rbegin()->first;
        candidate_map[stats.frame_range.first] = stats;
    }
    std::list<metrics::candidate_map_stats> candidate_list;
    for (auto const& candidate : candidate_map)
        candidate_list.push_back(candidate.second);
    stella_vslam_bfx::metrics::get_instance()->candidate_map_list = candidate_list;

    // Return a stored map if restoring, otherwise nothing
    if (restoring_first_stored_map)
        return stored_binary_maps.rbegin()->second.binary_map;
    return {};
#else
    return {};
#endif
}

//bool map_selector::have_better_map(data::map_database& map_db)
//{
//#if STORE_BINARY_MAPS
//    if (!stored_binary_maps.empty()) {
//        auto const best_stored = stored_binary_maps.rbegin();
//        if (map_db.get_num_keyframes() < best_stored->first)
//            return true;
//    }
//#else
//    unsigned int current_map_keyframe_count = map_db.get_num_keyframes();
//    for (auto& map : stored_maps)
//        if (map.get_num_keyframes() > current_map_keyframe_count)
//            return true;
//#endif
//    return false;
//}

void map_selector::set_system(std::shared_ptr<stella_vslam::system> slam)
{
    slam_ = slam;
}

} // namespace stella_vslam_bfx