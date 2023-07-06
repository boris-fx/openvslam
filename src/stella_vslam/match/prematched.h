#ifndef STELLA_VSLAM_BFX_PREMATCHED_H
#define STELLA_VSLAM_BFX_PREMATCHED_H

#include "stella_vslam/type.h"

#include <vector>

namespace stella_vslam {
namespace data {
class frame;
class keyframe;
class landmark;
} // namespace data
} // namespace stella_vslam

namespace stella_vslam_bfx {

// Equivalent to area::match_in_consistent_area()
unsigned int get_frames_prematches(stella_vslam::data::frame& frm_1, stella_vslam::data::frame& frm_2,
                                    std::vector<cv::Point2f>& prev_matched_pts,
                                    std::vector<int>& matched_indices_2_in_frm_1,
                                    bool check_orientation = false);

// Equivalent to match_frame_and_keyframe() for bow_tree/robust
unsigned int get_frame_and_keyframe_prematches(const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm,
                                                stella_vslam::data::frame& frm,
                                                std::vector<std::shared_ptr<stella_vslam::data::landmark>>& matched_lms_in_frm,
                                                bool check_orientation = false);

// Equivalent to match_keyframes() for bow_tree/robust
// Would be used only by the loop detector; may not be necessary?
/* unsigned int get_keyframes_prematches(const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_1,
                                const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_2,
                                std::vector<std::shared_ptr<stella_vslam::data::landmark>>& matched_lms_in_keyfrm_1); */

// Equivalent to robust::match_for_triangulation()
unsigned int get_prematches_for_triangulation(const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_1,
                                                const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm_2,
                                                const stella_vslam::Mat33_t& E_12,
                                                std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                                bool check_orientation = false);

// Equivalent to projection::match_frame_and_landmarks()
unsigned int add_prematched_landmarks(stella_vslam::data::frame& frm,
                                        const std::vector<std::shared_ptr<stella_vslam::data::landmark>>& landmarks,
                                        stella_vslam::eigen_alloc_unord_map<unsigned int, stella_vslam::Vec2_t>& lm_to_reproj,
                                        const float margin);

// Equivalent to projection::match_current_and_last_frames()
unsigned int add_frames_prematches(stella_vslam::data::frame& curr_frm, const stella_vslam::data::frame& last_frm,
                                        const float margin);

// Equivalent to projection::match_frame_and_keyframe()
unsigned int add_keyframe_prematches(stella_vslam::data::frame& curr_frm,
                                        const std::shared_ptr<stella_vslam::data::keyframe>& keyfrm,
                                        const std::set<std::shared_ptr<stella_vslam::data::landmark>>& already_matched_lms,
                                        const float margin);

} // namespace stella_vslam_bfx

#endif // STELLA_VSLAM_BFX_PREMATCHED_H