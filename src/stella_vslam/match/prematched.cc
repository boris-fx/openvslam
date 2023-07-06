#include "stella_vslam/match/prematched.h"
#include "stella_vslam/match/robust.h"
#include "stella_vslam/data/frame.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/util/angle.h"

using namespace stella_vslam;

namespace stella_vslam_bfx {

namespace {

// Get the index in the frame_observation of (key)frame 2 that corresponds to the prematched point at 
// idx_1 in the frame_observation of (key)frame 1
template <class S, class T>
int get_corresponding_index(int idx_1, const S& frm_1, const T& frm_2) {
    assert(idx_1 >= 0);
    const auto& obs_1 = frm_1.frm_obs_;
    const auto& obs_2 = frm_2.frm_obs_;
    auto it = obs_2.prematched_id_to_idx_.find(obs_1.prematched_idx_to_id_.at(idx_1));
    int idx_2 = (it == obs_2.prematched_id_to_idx_.end() ? -1 : it->second);

    assert(idx_2 < 0 || !frm_1.get_landmarks()[idx_1] || !frm_2.get_landmarks()[idx_2]
        || frm_1.get_landmarks()[idx_1]->prematched_id_ == frm_2.get_landmarks()[idx_2]->prematched_id_);
    return idx_2;
}

// Checks whether the distance between two points is less than the margin
// TODO better metric?
inline bool check_reprojection_error(const float x1, const float y1, const float x2, const float y2, const float margin) {
    float error_x = x1 - x2,
          error_y = y1 - y2;
    return std::sqrt(error_x * error_x + error_y * error_y) < margin;
}

} // anonymous namespace

unsigned int get_frames_prematches(data::frame& frm_1, data::frame& frm_2,
                                    std::vector<cv::Point2f>& prev_matched_pts,
                                    std::vector<int>& matched_indices_2_in_frm_1,
                                    bool check_orientation) {
    unsigned int num_matches = 0;

    // Output array may have been filled with ORB results,
    // so only overwrite values at prematched indices
    matched_indices_2_in_frm_1.resize(frm_1.frm_obs_.undist_keypts_.size(), -1);

    const auto keypts_1 = frm_1.frm_obs_.undist_keypts_;
    const auto keypts_2 = frm_2.frm_obs_.undist_keypts_;
    
    for (int idx_1 = frm_1.frm_obs_.prematched_keypts_.first; idx_1 < frm_1.frm_obs_.prematched_keypts_.second; ++idx_1) {
        auto idx_2 = get_corresponding_index(idx_1, frm_1, frm_2);

        // results may not have been cleared
        matched_indices_2_in_frm_1.at(idx_1) = idx_2;
        if ( 0 <= idx_2 ) {
            if (check_orientation && std::abs(util::angle::diff(keypts_1.at(idx_1).angle, keypts_2.at(idx_2).angle)) > 30.0) {
                continue;
            }

            prev_matched_pts.at(idx_1) = frm_2.frm_obs_.undist_keypts_.at(idx_2).pt;
            ++num_matches;
        }
    }

    return num_matches;
}

unsigned int get_frame_and_keyframe_prematches(const std::shared_ptr<data::keyframe>& keyfrm, data::frame& frm,
                                                std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm,
                                                bool check_orientation) {
    unsigned int num_matches = 0;
    const auto keyfrm_lms = keyfrm->get_landmarks();
    
    // Output array may have been filled with ORB results,
    // so only overwrite values at prematched indices
    matched_lms_in_frm.resize(frm.frm_obs_.num_keypts_, nullptr);

    const auto keypts_frm = frm.frm_obs_.undist_keypts_;
    const auto keypts_keyfrm = keyfrm->frm_obs_.undist_keypts_;
    
    for (int keyfrm_idx = keyfrm->frm_obs_.prematched_keypts_.first; keyfrm_idx < keyfrm->frm_obs_.prematched_keypts_.second; ++keyfrm_idx) {
        auto& lm = keyfrm_lms.at(keyfrm_idx);
        if ( lm != nullptr && !lm->will_be_erased() ) {

            auto frm_idx = get_corresponding_index(keyfrm_idx, *keyfrm, frm);
            if ( 0 <= frm_idx ) {
                if (check_orientation && std::abs(util::angle::diff(keypts_frm.at(frm_idx).angle, keypts_keyfrm.at(keyfrm_idx).angle)) > 30.0) {
                    continue;
                }
                
                matched_lms_in_frm.at(frm_idx) = lm;
                ++num_matches;
            }
        }
    }

    return num_matches;
}

/* In case we want to use the loop detector...
unsigned int get_keyframes_prematches(const std::shared_ptr<data::keyframe>& keyfrm_1,
                                const std::shared_ptr<data::keyframe>& keyfrm_2,
                                std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_1) {
    unsigned int num_matches = 0;

    const auto keyfrm_1_lms = keyfrm_1->get_landmarks();
    const auto keyfrm_2_lms = keyfrm_2->get_landmarks();
    matched_lms_in_keyfrm_1.resize(keyfrm_1_lms.size(), nullptr);

    for (int idx_1 = keyfrm_1->frm_obs_.prematched_keypts_.first; idx_1 < keyfrm_1->frm_obs_.prematched_keypts_.second; ++idx_1) {
        auto& lm_1 = keyfrm_1_lms.at(idx_1);
        if ( lm_1 != nullptr && !lm_1->will_be_erased() ) {

            auto idx_2 = get_corresponding_index(idx_1, *keyfrm_1, *keyfrm_2);
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

unsigned int get_prematches_for_triangulation(const std::shared_ptr<data::keyframe>& keyfrm_1,
                                                const std::shared_ptr<data::keyframe>& keyfrm_2,
                                                const Mat33_t& E_12,
                                                std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                                bool check_orientation) {
    unsigned int num_matches = 0;

    // Project the center of keyframe 1 to keyframe 2
    // to acquire the epipole coordinates of the candidate keyframe
    const Vec3_t cam_center_1 = keyfrm_1->get_trans_wc();
    const Mat33_t rot_2w = keyfrm_2->get_rot_cw();
    const Vec3_t trans_2w = keyfrm_2->get_trans_cw();
    Vec3_t epiplane_in_keyfrm_2;
    const bool valid_epiplane = keyfrm_2->camera_->reproject_to_bearing(rot_2w, trans_2w, cam_center_1, epiplane_in_keyfrm_2);

    // Acquire the 3D point information of the keframes
    const auto lms_in_keyfrm_1 = keyfrm_1->get_landmarks();
    const auto lms_in_keyfrm_2 = keyfrm_2->get_landmarks();

    // Output array may have been filled with ORB results,
    // so only overwrite values at prematched indices
    matched_idx_pairs.reserve( matched_idx_pairs.size() + keyfrm_1->frm_obs_.num_prematched_points() );

    // TODO: filter out keypoints near the epipole? (Requires bearing vector)
    for (int idx_1 = keyfrm_1->frm_obs_.prematched_keypts_.first; idx_1 < keyfrm_1->frm_obs_.prematched_keypts_.second; ++idx_1) {
        if (lms_in_keyfrm_1.at(idx_1)) {
            continue;
        }

        // Ignore if the keypoint is associated with any 3D points
        // (because this function is used for triangulation)
        const auto idx_2 = get_corresponding_index(idx_1, *keyfrm_1, *keyfrm_2);
        if (idx_2 < 0 || lms_in_keyfrm_2.at(idx_2)) {
            continue;
        }

        const auto& keypt_1 = keyfrm_1->frm_obs_.undist_keypts_.at(idx_1);
        const Vec3_t& bearing_1 = keyfrm_1->frm_obs_.bearings_.at(idx_1);
        const auto& keypt_2 = keyfrm_2->frm_obs_.undist_keypts_.at(idx_2);
        const Vec3_t& bearing_2 = keyfrm_2->frm_obs_.bearings_.at(idx_2);

        if (check_orientation && std::abs(util::angle::diff(keypt_1.angle, keypt_2.angle)) > 30.0) {
            continue;
        }

        // Do not use any keypoints near the epipole
        if (valid_epiplane) {
            const auto cos_dist = epiplane_in_keyfrm_2.dot(bearing_2);
            // The threshold of the minimum angle formed by the epipole and the bearing vector is 3.0 degree
            constexpr double cos_dist_thr = 0.99862953475;

            // Do not allow to match if the formed angle is narrower that the threshold value
            if (cos_dist_thr < cos_dist) {
                continue;
            }
        }

        // Check consistency in Matrix E
        const bool is_inlier = match::robust::check_epipolar_constraint(bearing_1, bearing_2, E_12,
                                                keyfrm_1->orb_params_->scale_factors_.at(keypt_1.octave));
            
        if (is_inlier) {
            matched_idx_pairs.emplace_back( std::make_pair(idx_1, idx_2) );
            ++num_matches;
        }
    }

    return num_matches;
}

unsigned int add_prematched_landmarks(data::frame& frm,
                                        const std::vector<std::shared_ptr<data::landmark>>& local_landmarks,
                                        eigen_alloc_unord_map<unsigned int, Vec2_t>& lm_to_reproj,
                                        const float margin) {
    unsigned int num_matches = 0;

    for (auto local_lm : local_landmarks) {
        if (!local_lm) {
            continue;
        }
        if (!lm_to_reproj.count(local_lm->id_)) {
            continue;
        }
        if (local_lm->will_be_erased()) {
            continue;
        }
        if (local_lm->prematched_id_ < 0) {
            continue;
        }

        auto it = frm.frm_obs_.prematched_id_to_idx_.find(local_lm->prematched_id_);
        if (it == frm.frm_obs_.prematched_id_to_idx_.end()) {
            continue;
        }

        auto idx = it->second;
        const auto& lm = frm.get_landmark(idx);
        if (lm && lm->has_observation()) {
            continue;
        }

        // Check the reprojected landmark is reasonably close to the keypoint location
        const auto reproj = lm_to_reproj.at(local_lm->id_);
        const auto keypt = frm.frm_obs_.undist_keypts_.at(idx).pt;
        if (!check_reprojection_error(reproj(0), reproj(1), keypt.x, keypt.y, margin))
            continue;

        frm.add_landmark(local_lm, idx);
        ++num_matches;
    }

    return num_matches;
}

unsigned int add_frames_prematches(data::frame& curr_frm, const data::frame& last_frm, const float margin) {
    unsigned int num_matches = 0;

    const Mat33_t rot_cw = curr_frm.get_rot_cw();
    const Vec3_t trans_cw = curr_frm.get_trans_cw();

    for (int idx_last = last_frm.frm_obs_.prematched_keypts_.first;
            idx_last < last_frm.frm_obs_.prematched_keypts_.second;
            ++idx_last) {
        const auto& lm = last_frm.landmarks_.at(idx_last);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        assert(lm->prematched_id_ >= 0);

        const auto idx_curr = get_corresponding_index(idx_last, last_frm, curr_frm);
        if (idx_curr < 0) {
            continue;
        }

        const auto& curr_lm = curr_frm.get_landmark(idx_curr);
        if (curr_lm && curr_lm->has_observation()) {
            continue;
        }
        assert(!curr_lm || curr_lm->prematched_id_ >= 0);

        // 3D point coordinates with the global reference
        const Vec3_t pos_w = lm->get_pos_in_world();

        // Reproject and compute visibility
        Vec2_t reproj;
        float x_right;
        const bool in_image = curr_frm.camera_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj, x_right);

        // Ignore if it is reprojected outside the image
        if (!in_image) {
            continue;
        }

        // Check the reprojected landmark is reasonably close to the keypoint location
        const auto keypt = curr_frm.frm_obs_.undist_keypts_.at(idx_curr).pt;
        if (!check_reprojection_error(reproj(0), reproj(1), keypt.x, keypt.y, margin))
            continue;

        // The matching is valid
        curr_frm.add_landmark(lm, idx_curr);
        ++num_matches;
    }

    return num_matches;
}

unsigned int add_keyframe_prematches(data::frame& frm,
                                        const std::shared_ptr<data::keyframe>& keyfrm,
                                        const std::set<std::shared_ptr<data::landmark>>& already_matched_lms,
                                        const float margin) {
    unsigned int num_matches = 0;

    const Mat33_t rot_cw = frm.get_pose_cw().block<3, 3>(0, 0);
    const Vec3_t trans_cw = frm.get_pose_cw().block<3, 1>(0, 3);

    for (int idx = keyfrm->frm_obs_.prematched_keypts_.first;
            idx < keyfrm->frm_obs_.prematched_keypts_.second;
            ++idx) {

        auto& lm = keyfrm->get_landmark(idx);
        if (!lm) { 
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // Avoid duplication
        if (already_matched_lms.count(lm)) {
            continue;
        }

        assert(lm->prematched_id_ >= 0);

        const auto frm_idx = get_corresponding_index(idx, *keyfrm, frm);
        if (frm_idx < 0) {
            continue;
        }

        if (frm.get_landmark(frm_idx)) {
            continue;
        }

        // 3D point coordinates with the global reference
        const Vec3_t pos_w = lm->get_pos_in_world();

        // Reproject and compute visibility
        Vec2_t reproj;
        float x_right;
        const bool in_image = frm.camera_->reproject_to_image(rot_cw, trans_cw, pos_w, reproj, x_right);

        // Ignore if it is reprojected outside the image
        if (!in_image) {
            continue;
        }

        // Check the reprojected landmark is reasonably close to the keypoint location
        const auto keypt = frm.frm_obs_.undist_keypts_.at(frm_idx).pt;
        if (!check_reprojection_error(reproj(0), reproj(1), keypt.x, keypt.y, margin))
            continue;

        // The matching is valid
        frm.add_landmark(lm, frm_idx);
        ++num_matches;
    }

    return num_matches;
}

} // namespace stella_vslam_bfx