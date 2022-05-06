#include "stella_vslam/match/prematched.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"

namespace stella_vslam_bfx {

// Get the index in obs_2 that corresponds to the prematched point at idx_1 in obs_1
inline int get_corresponding_index(int idx_1, const stella_vslam::data::frame_observation& obs_1,
                                    const stella_vslam::data::frame_observation& obs_2) {
    auto it = obs_2.prematched_id_to_idx_.find(obs_1.prematched_idx_to_id_.at(idx_1));
    return it == obs_2.prematched_id_to_idx_.end() ? -1 : it->second;
}

unsigned int get_frames_prematches(stella_vslam::data::frame& frm_1, stella_vslam::data::frame& frm_2,
                            std::vector<cv::Point2f>& prev_matched_pts, std::vector<int>& matched_indices_2_in_frm_1) {
    unsigned int num_matches = 0;

    // Output array may have been filled with ORB results,
    // so only overwrite values at prematched indices
    matched_indices_2_in_frm_1.resize(frm_1.frm_obs_.undist_keypts_.size(), -1);
    
    for (int idx_1 = frm_1.frm_obs_.prematched_keypts_.first; idx_1 < frm_1.frm_obs_.prematched_keypts_.second; ++idx_1) {
        auto idx_2 = get_corresponding_index(idx_1, frm_1.frm_obs_, frm_2.frm_obs_);

        // results may not have been cleared
        matched_indices_2_in_frm_1.at(idx_1) = idx_2;
        if ( 0 <= idx_2 ) {
            prev_matched_pts.at(idx_1) = frm_2.frm_obs_.undist_keypts_.at(idx_2).pt;
            ++num_matches;
        }
    }

    return num_matches;
}

unsigned int get_frames_prematches(stella_vslam::data::frame& curr_frm, const stella_vslam::data::frame& last_frm) {
    unsigned int num_matches = 0;

    for (int idx_last = last_frm.frm_obs_.prematched_keypts_.first; idx_last < last_frm.frm_obs_.prematched_keypts_.second; ++idx_last) {
        auto& lm = last_frm.landmarks_.at(idx_last);
        if (lm != nullptr){
            
            auto idx_curr = get_corresponding_index(idx_last, last_frm.frm_obs_, curr_frm.frm_obs_);
            if ( 0 <= idx_curr  &&
                    !(curr_frm.landmarks_.at(idx_curr) && curr_frm.landmarks_.at(idx_curr)->has_observation()) ) {
                curr_frm.landmarks_.at(idx_curr) = lm;
                ++num_matches;
            }
        }
    }

    return num_matches;
}

unsigned int get_frame_and_keyframe_prematches(const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm, stella_vslam::data::frame& frm,
                                                std::vector<std::shared_ptr<stella_vslam::data::landmark>>& matched_lms_in_frm) {
    unsigned int num_matches = 0;
    const auto keyfrm_lms = keyfrm->get_landmarks();
    
    // Output array may have been filled with ORB results,
    // so only overwrite values at prematched indices
    matched_lms_in_frm.resize(frm.frm_obs_.num_keypts_, nullptr);
    
    for (int keyfrm_idx = keyfrm->frm_obs_.prematched_keypts_.first; keyfrm_idx < keyfrm->frm_obs_.prematched_keypts_.second; ++keyfrm_idx) {
        auto& lm = keyfrm_lms.at(keyfrm_idx);
        if ( lm != nullptr && !lm->will_be_erased() ) {

            auto frm_idx = get_corresponding_index(keyfrm_idx, keyfrm->frm_obs_, frm.frm_obs_);
            if ( 0 <= frm_idx ) {
                matched_lms_in_frm.at(frm_idx) = lm;
                ++num_matches;
            }
        }
    }

    return num_matches;
}

/* In case we want to use the loop detector...
unsigned int get_keyframes_prematches(const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_1,
                                const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_2,
                                std::vector<std::shared_ptr<stella_vslam::data::landmark>>& matched_lms_in_keyfrm_1) {
    unsigned int num_matches = 0;

    const auto keyfrm_1_lms = keyfrm_1->get_landmarks();
    const auto keyfrm_2_lms = keyfrm_2->get_landmarks();
    matched_lms_in_keyfrm_1.resize(keyfrm_1_lms.size(), nullptr);

    for (int idx_1 = keyfrm_1->frm_obs_.prematched_keypts_.first; idx_1 < keyfrm_1->frm_obs_.prematched_keypts_.second; ++idx_1) {
        auto& lm_1 = keyfrm_1_lms.at(idx_1);
        if ( lm_1 != nullptr && !lm_1->will_be_erased() ) {

            auto idx_2 = get_corresponding_index(idx_1, keyfrm_1->frm_obs_, keyfrm_2->frm_obs_);
            if ( 0 <= idx_2 ) {
                auto& lm_2 = keyfrm_2_lms.at(idx_2);
                if ( lm_2 != nullptr && !lm_2->will_be_erased() ) {
                    matched_lms_in_keyfrm_1.at(idx_1) = lm_2;
                    ++num_matches;
                }
            }
        }
    }

    return num_matches;
} */

unsigned int get_prematches_for_triangulation(const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_1,
                                                const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_2,
                                                std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs) {
    unsigned int num_matches = 0;

    const auto lms_in_keyfrm_1 = keyfrm_1->get_landmarks();
    const auto lms_in_keyfrm_2 = keyfrm_2->get_landmarks();

    // Output array may have been filled with ORB results,
    // so only overwrite values at prematched indices
    matched_idx_pairs.reserve( matched_idx_pairs.size() + keyfrm_1->frm_obs_.num_prematched_points() );

    // TODO: filter out keypoints near the epipole? (Requires bearing vector)
    for (int idx_1 = keyfrm_1->frm_obs_.prematched_keypts_.first; idx_1 < keyfrm_1->frm_obs_.prematched_keypts_.second; ++idx_1) {
        if ( !lms_in_keyfrm_1.at(idx_1) )
        {
            auto idx_2 = get_corresponding_index(idx_1, keyfrm_1->frm_obs_, keyfrm_2->frm_obs_);
            if ( 0 <= idx_2 && !lms_in_keyfrm_2.at(idx_2) ) {
                matched_idx_pairs.emplace_back( std::make_pair(idx_1, idx_2) );
                ++num_matches;
            }
        }
    }

    return num_matches;
}
} // namespace stella_vslam_bfx