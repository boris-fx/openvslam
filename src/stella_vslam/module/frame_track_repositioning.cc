#include "frame_track_repositioning.h"

#include <tuple>
#include <memory>
#include <optional>

#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/map_database.h>
#include <stella_vslam/tracking_module.h>
#include <stella_vslam/system.h>

#include <stella_vslam/report/metrics.h>

using namespace stella_vslam;

namespace stella_vslam_bfx {

static bool earliest_valid_keyframe(std::vector<std::shared_ptr<data::keyframe>> const& keyfrms,
                                    std::map<double, stage_and_frame> const& timestamp_to_stage_and_frame,
                                    std::shared_ptr<data::keyframe> &keyframe,
                                    std::optional<int> &keyframes_frame_number) {

    if (!keyfrms.empty()) {
        std::shared_ptr<data::keyframe> first_keyframe_data = *std::min_element(keyfrms.begin(), keyfrms.end(),
            [](const auto& a, const auto& b) { // earliest above landmark threshold
                int landmarkThreshold(10);
                bool validLandmarksA(a->get_valid_landmarks().size() >= landmarkThreshold);
                bool validLandmarksB(b->get_valid_landmarks().size() >= landmarkThreshold);
                if (!validLandmarksA && validLandmarksB)
                    return false;
                if (validLandmarksA && !validLandmarksB)
                    return true;
                return a->timestamp_ < b->timestamp_;
            });
        auto f = timestamp_to_stage_and_frame.find(first_keyframe_data->timestamp_);
        if (f != timestamp_to_stage_and_frame.end()) {
            keyframe = first_keyframe_data;
            keyframes_frame_number = f->second.frame;
            return true;
        }
    }
    keyframe.reset();
    keyframes_frame_number = std::nullopt;
    return false;
}

bool reposition_tracking_to_first_map_keyframe(std::shared_ptr<stella_vslam::system> slam,
                                               std::map<double, stage_and_frame> const& timestamp_to_stage_and_frame,
                                               int &reposition_frame)
{
    // This function will record the pass_1_end_keyframes and pass_2_frames (on sucess) metrics values
    metrics& track_metrics = *metrics::get_instance();
    auto map_keyframes = slam->map_db_->get_all_keyframes();

    // Find the first keyframe in the map
    std::shared_ptr<data::keyframe> earliest_keyframe;
    std::optional<int>              earliest_keyframes_frame_number;
    bool found_earliest_keyframe = earliest_valid_keyframe(map_keyframes, timestamp_to_stage_and_frame, earliest_keyframe, earliest_keyframes_frame_number);
    if (found_earliest_keyframe) {
        track_metrics.pass_1_end_keyframes = map_keyframes.size();
        //track_metrics.pass_2_frames = ; // set this in solver, which decides to start on the reposition frame, or the one before
        reposition_frame = earliest_keyframes_frame_number.value();
    }
    else {
        spdlog::error("Failed to create any valid keyframes during initialialisation");
        track_metrics.pass_1_end_keyframes = 0;
        track_metrics.pass_2_frames = 0;
        reposition_frame = -1;
        return false;
    }

    bool const relocalize_by_pose(false);
    if (relocalize_by_pose) {
        // Relocalise the tracker to the first keyframe's pose
        bool resultRelocalise = slam->relocalize_by_pose(earliest_keyframe->get_pose_wc());
        if (resultRelocalise == true)
            spdlog::info("Relocalised to first keyframe pose (video frame {})", earliest_keyframes_frame_number.value());
        else {
            spdlog::error("Failed to relocalise to first keyframe pose (video frame {})", earliest_keyframes_frame_number.value());
            return false;
        }
    }

    else {

        // Set the tracker to 'lost' and send the first keyframe to the tracker - check that it finds the matches correctly as would be expected
        slam->tracker_->tracking_state_ = tracker_state_t::Lost;

            

    }

    slam->tracker_->last_frm_.ref_keyfrm_ = earliest_keyframe;

    return true;
}


} // namespace stella_vslam_bfx