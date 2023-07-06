#ifndef STELLA_VSLAM_DATA_FRAME_OBSERVATION_H
#define STELLA_VSLAM_DATA_FRAME_OBSERVATION_H

#include "stella_vslam/type.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace stella_vslam {
namespace data {

struct frame_observation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame_observation() = default;
    frame_observation(unsigned int num_keypts, const cv::Mat& descriptors,
                      const std::vector<cv::KeyPoint>& undist_keypts, const eigen_alloc_vector<Vec3_t>& bearings,
                      const std::vector<float>& stereo_x_right, const std::vector<float>& depths,
                      const std::vector<std::vector<std::vector<unsigned int>>>& keypt_indices_in_cells)
        : num_keypts_(num_keypts), descriptors_(descriptors), undist_keypts_(undist_keypts), bearings_(bearings),
          stereo_x_right_(stereo_x_right), depths_(depths), keypt_indices_in_cells_(keypt_indices_in_cells) {}

    //! number of keypoints
    unsigned int num_keypts_ = 0;
    //! descriptors
    cv::Mat descriptors_;
    //! undistorted keypoints of monocular or stereo left image
    std::vector<cv::KeyPoint> undist_keypts_;
    //! bearing vectors
    eigen_alloc_vector<Vec3_t> bearings_;
    //! disparities
    std::vector<float> stereo_x_right_;
    //! depths
    std::vector<float> depths_;
    //! keypoint indices in each of the cells
    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells_;

    //-----------------------------------------
    // stella_vslam_bfx modification for importing tracked point matches

    //! index range of prematched keypoints (first is inclusive, second is exclusive)
    std::pair<int, int> prematched_keypts_ = std::make_pair(-1, -1);
    //! ID of each prematched point by index (persists across frames)
    std::unordered_map<unsigned, unsigned> prematched_idx_to_id_;
    //! find the index of a prematched point in this frame from its ID
    std::unordered_map<unsigned, unsigned> prematched_id_to_idx_;

    //! Does the given index belong to a prematched keypoint?
    bool idx_is_prematched(const int idx) const {
        return 0 <= prematched_keypts_.first && idx >= prematched_keypts_.first && idx < prematched_keypts_.second;
    }

    unsigned int num_prematched_points() const {
        return 0 > prematched_keypts_.first ? 0 : prematched_keypts_.second - prematched_keypts_.first;
    }

};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_FRAME_OBSERVATION_H
