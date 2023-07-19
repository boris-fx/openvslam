#include "stella_vslam/feature/orb_preanalysis.h"

#include "stella_vslam/system.h"
#include "stella_vslam/data/frame.h"

//#include "stella_vslam/config.h"
//#include "stella_vslam/data/frame_observation.h"
//#include "stella_vslam/util/image_converter.h"
#include "stella_vslam/match/area.h"
//
//#include "orb_extractor.h"
//
// #include <opencv2/core/mat.hpp>
//#include <opencv2/core/mat.hpp>
//#include <opencv2/core/types.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/features2d.hpp>
//
//#include <iostream>
//
//#include <spdlog/spdlog.h>

namespace stella_vslam_bfx{


    // take first, last and centre frame + 1 every 10 sec (1000 frames-ish)
    // Check for variation
    // 
    // Create a keyframed parameter vs
    // 
    // Check at start, then monitor and adjustas we go through the video, especially at each reset
    // Num features, num scale 0 features, num matches
    // after init num scale 0 features may be less relevant??

void preanalysis(//stella_vslam::config const& cfg,
                 const cv::Mat& img_1, const cv::Mat& mask_1,
                 const cv::Mat& img_2, const cv::Mat& mask_2,
                 stella_vslam::system * sys) {

    int min_matches(100);

    float boost(1.0f);
    for (int i = 0; i < 10; ++i) {

        stella_vslam::data::frame frame_1 = sys->create_monocular_frame(img_1, 0.0, mask_1);
        stella_vslam::data::frame frame_2 = sys->create_monocular_frame(img_2, 1.0, mask_2);
        
        // Match using the same code as in initializer::try_initialize_for_monocular
        stella_vslam::match::area matcher(0.9, true);
        std::vector<cv::Point2f> prev_matched_coords;
        std::vector<int> init_matches;

        // initialize the previously matched coordinates
        prev_matched_coords.resize(frame_1.frm_obs_.undist_keypts_.size());
        for (unsigned int i = 0; i < frame_1.frm_obs_.undist_keypts_.size(); ++i) {
            prev_matched_coords.at(i) = frame_1.frm_obs_.undist_keypts_.at(i).pt;
        }

        unsigned int num_matches = matcher.match_in_consistent_area(frame_1, frame_2, prev_matched_coords, init_matches, 100);

        spdlog::info("solver - boost check matches {} {}->{}", frame_1.frm_obs_.undist_keypts_.size(), frame_2.frm_obs_.undist_keypts_.size(), num_matches);

        if (num_matches >= min_matches)
            break;

        boost *= 0.9f;
        sys->boost_extractors(boost);
    }

    int yy = 0;
}

#if 0
        // Setup the feature extractor
        stella_vslam::feature::orb_params orb_params(cfg.settings_);
        auto mask_rectangles = cfg.settings_.mask_rectangles_;
        const auto min_size = cfg.settings_.min_feature_size_;
        stella_vslam::feature::orb_extractor extractor(&orb_params, min_size, mask_rectangles);

        // Extract features from frame 1
        cv::Mat img_gray_1 = img_1;
        stella_vslam::util::convert_to_grayscale(img_gray_1, camera->color_order_);
        stella_vslam::data::frame_observation frm_obs_1;
        std::vector<cv::KeyPoint> keypts_1;
        extractor.extract(img_gray_1, mask_1, keypts_1, frm_obs_1.descriptors_);
        frm_obs_1.num_keypts_ = keypts_1.size();
        camera->undistort_keypoints(keypts_1, frm_obs_1.undist_keypts_);


        return data::frame(timestamp, camera_, orb_params_, frm_obs, std::move(markers_2d));

    }

    frm_1.frm_obs_.undist_keypts_

    // Match using the same code as in initializer::try_initialize_for_monocular
    stella_vslam::match::area matcher(0.9, true);
    //stella_vslam_bfx::metrics::get_instance()->capture_area_matching = true;
    unsigned int num_matches = matcher.match_in_consistent_area(init_frm_, curr_frm, prev_matched_coords_, init_matches_, 100);
    //stella_vslam_bfx::metrics::get_instance()->capture_area_matching = false;
#endif


} // namespace stella_vslam_bfx
