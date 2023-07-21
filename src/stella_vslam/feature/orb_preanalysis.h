#ifndef STELLA_VSLAM_FEATURE_ORB_PREANALYSIS_H
#define STELLA_VSLAM_FEATURE_ORB_PREANALYSIS_H

//#include "stella_vslam/feature/orb_params.h"
//#include "stella_vslam/feature/orb_extractor_node.h"
//#include "stella_vslam/feature/orb_impl.h"
//
//#include <opencv2/core/mat.hpp>
//#include <opencv2/core/types.hpp>
#include <memory>
#include <map>
#include <utility>

namespace stella_vslam { class system; class config; }
namespace stella_vslam::feature { class orb_extractor; }
namespace cv { class Mat; }

namespace stella_vslam_bfx {
 
    bool use_feature_monitor();

    void preanalysis(//stella_vslam::config const& cfg,
        const cv::Mat& img_1, const cv::Mat& mask_1,
        const cv::Mat& img_2, const cv::Mat& mask_2,
        stella_vslam::system* sys);

    class orb_feature_monitor
    {
    public:

        orb_feature_monitor(unsigned int target_feature_count = 2000);


        /// Get an initial min_feature_size based only on video dimensions
        static unsigned int default_min_feature_size_for_video_size(int width, int height);

        /// 
        void update_feature_extractor(const cv::Mat& img, const cv::Mat& mask, stella_vslam::feature::orb_extractor* extractor);

        /// 
        void record_extraction_result(unsigned int feature_count, stella_vslam::feature::orb_extractor* extractor);

    protected:


        bool initial_min_feature_size_multiplier_set_;

        unsigned int base_min_feature_size_;

        unsigned int target_feature_count_;

        std::map<int, std::pair<float, int>> multiplier_and_count_by_frame_;

        int   latest_recorded_frame_;
        float multiplier_for_latest_recorded_frame_;
    };

} // namespace stella_vslam_bfx

#endif // STELLA_VSLAM_FEATURE_ORB_PREANALYSIS_H
