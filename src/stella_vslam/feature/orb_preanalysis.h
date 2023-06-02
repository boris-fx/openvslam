#ifndef STELLA_VSLAM_FEATURE_ORB_PREANALYSIS_H
#define STELLA_VSLAM_FEATURE_ORB_PREANALYSIS_H

//#include "stella_vslam/feature/orb_params.h"
//#include "stella_vslam/feature/orb_extractor_node.h"
//#include "stella_vslam/feature/orb_impl.h"
//
//#include <opencv2/core/mat.hpp>
//#include <opencv2/core/types.hpp>

namespace stella_vslam {  class system; }
namespace cv { class Mat; }

namespace stella_vslam_bfx {
 
    void preanalysis(//stella_vslam::config const& cfg,
        const cv::Mat& img_1, const cv::Mat& mask_1,
        const cv::Mat& img_2, const cv::Mat& mask_2,
        stella_vslam::system* sys);

} // namespace stella_vslam_bfx

#endif // STELLA_VSLAM_FEATURE_ORB_PREANALYSIS_H
