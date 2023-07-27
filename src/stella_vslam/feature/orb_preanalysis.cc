#include "stella_vslam/feature/orb_preanalysis.h"

#include "stella_vslam/system.h"
#include "stella_vslam/data/frame.h"

//#include "stella_vslam/config.h"
//#include "stella_vslam/data/frame_observation.h"
//#include "stella_vslam/util/image_converter.h"
#include "stella_vslam/match/area.h"
#include "stella_vslam/config.h"
#include "stella_vslam/report/metrics.h"
#include "stella_vslam/report/plot_html.h"
#include "stella_vslam/feature/orb_extractor.h"
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

    bool use_feature_monitor() {

        return false;

    }

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

    float multiplier(1.0f);
    for (int i = 0; i < 10; ++i) {

        stella_vslam::data::frame frame_1 = sys->create_monocular_frame(img_1, 0.0, mask_1);
        stella_vslam::data::frame frame_2 = sys->create_monocular_frame(img_2, 1.0, mask_2);
        
        // Match using the same code as in initializer::try_initialize_for_monocular
        stella_vslam::match::area matcher(0.9, true);
        std::vector<cv::Point2f> prev_matched_coords;
        std::vector<int> init_matches;

        // initialize the previously matched coordinates
        prev_matched_coords.resize(frame_1.frm_obs_.undist_keypts_.size());
        for (unsigned int k = 0; k < frame_1.frm_obs_.undist_keypts_.size(); ++k) {
            prev_matched_coords.at(k) = frame_1.frm_obs_.undist_keypts_.at(k).pt;
        }

        unsigned int num_matches = matcher.match_in_consistent_area(frame_1, frame_2, prev_matched_coords, init_matches, 100);

        spdlog::info("solver - multiplier check matches {} {}->{}", frame_1.frm_obs_.undist_keypts_.size(), frame_2.frm_obs_.undist_keypts_.size(), num_matches);

        if ((int)num_matches >= min_matches)
            break;

        multiplier *= 0.9f;
        sys->boost_extractors(multiplier);
    }

    
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

orb_feature_monitor::orb_feature_monitor(unsigned int target_feature_count)
: target_feature_count_(target_feature_count)
{
}

double bisection(double a, double b, std::function<int(double)> const& func, double epsilon_ab)
{
    int fa = func(a);
    int fb = func(b);
    int fc;
    if (fa * fb >= 0)
        return std::abs(fa) < std::abs(fb) ? a : b; // a and b both give too few (or too many) features, return the nearest to the target

    double c = a;

    while (((b - a) >= epsilon_ab) || fa==fb) // stop if a and b are close enough, or fa==fb (not possible)
    {
        c = (a + b) / 2;
        fc = func(c);
        if (fc == 0) {
            return c; // perfect solution found (in c)
        }
        else if (fc * fa < 0) {
            // fa and fc have opposite sign, c becomes the new b
            b = c;
            fb = fc;
        }
        else {
            // fa and fc have the same sign, c becomes the new a
            a = c;
            fa = fc;
        }
    }
    return std::abs(fa) < std::abs(fb) ? a : b; // return the nearest to the target
}

void save_graph_min_feature_size_vs_feature_count(const cv::Mat& img, const cv::Mat& mask, stella_vslam::feature::orb_extractor* extractor, std::optional<double> target_feature_count)
{
    // Collect data points
    std::map<double, double> curve_min_feature_size, curve_min_feature_diameter;
    std::list<int> diameters;
    for (int i = 1; i < 20; ++i)
        diameters.push_back(i);
    for (int i = 22; i < 50; i+=2)
        diameters.push_back(i);
    for (int i = 50; i < 500; i+=5)
        diameters.push_back(i);
    for (auto const& diameter : diameters) {
        std::vector<cv::KeyPoint> keypts;
        cv::Mat descriptors;
        double final_min_feature_size(diameter * diameter);
        float min_feature_size_multiplier = final_min_feature_size / extractor->min_feature_size();
        extractor->min_size_multiplier_ = min_feature_size_multiplier;
        extractor->extract(img, mask, keypts, descriptors);
        curve_min_feature_size[diameter * diameter] = (double)keypts.size();
        curve_min_feature_diameter[diameter] = (double)keypts.size();
    }

    // Save the html file
    html_file html("min_feature_size_vs_feature_count.html");
    Graph vs_min_feature_diameter("Min feature diameter (pixels)", "Feature Count", std::set<Curve>({ {"Feature count", curve_min_feature_diameter} }),
        range_behaviour::no_max, range_behaviour::no_max, target_feature_count);
    write_graph_as_svg(html.html, vs_min_feature_diameter);
    Graph vs_min_feature_size("Min feature size (pixels)", "Feature Count", std::set<Curve>({ {"Feature count", curve_min_feature_size} }),
        range_behaviour::no_max, range_behaviour::no_max, target_feature_count);
    write_graph_as_svg(html.html, vs_min_feature_size);
}


template <typename T1, typename T2>
std::optional<T2> nearest_data_in_map(const std::map<T1, T2>& data, T1 key)
{
    if (data.size() == 0)
        return std::nullopt;

    auto lower = data.lower_bound(key);

    if (lower == data.end()) // If none found, return the last one.
        return std::prev(lower)->second;

    if (lower == data.begin())
        return lower->second;

    // Check which one is nearest.
    auto previous = std::prev(lower);
    if ((key - previous->first) < (lower->first - key))
        return previous->second;

    return lower->second;
}

void orb_feature_monitor::update_feature_extractor(const cv::Mat& img, const cv::Mat& mask, stella_vslam::feature::orb_extractor* extractor)
{
    if (!use_feature_monitor())
        return;
    if (!extractor)
        return;
    
    int const low_min_feature_size(9); // lowest acceptable feature area in pixels
    int const high_min_feature_size(80000); // highest acceptable feature area in pixels
    float const min_feature_diameter_step(2.0f);
    float const trigger_scale(1.15f); // Lower min_feature_size if feature count > trigger_scale * target feature count

    // Get the frame number (currently from the metrics)
    std::optional<stage_and_frame> stage_with_frame = metrics::instance()->timestamp_to_frame(metrics::instance()->current_frame_timestamp);
    if (!stage_with_frame)
        return;
    int frame = stage_with_frame->frame;

    // (i) First check for a historic frame match, use the multiplier if one is found
    auto frame_match = data_by_frame_.find(frame);
    if (frame_match != data_by_frame_.end()) {
        extractor->min_size_multiplier_ = frame_match->second.multiplier;
        return;
    }

    // (ii) Use a nearly frame, modifying the multiplier if necessary
    std::optional<frame_data> nearby_frame_data = last_frame_data_;
    if (!nearby_frame_data || std::abs(nearby_frame_data->frame - frame)>1)
        nearby_frame_data = nearest_data_in_map(data_by_frame_, frame);
    if (nearby_frame_data) {

        float min_size_multiplier = nearby_frame_data->multiplier;

        // Decrease the multiplier if too few features were detected last time
        int nearby_feature_count = nearby_frame_data->feature_count;
        if (float(nearby_feature_count) < (1.0f/trigger_scale) * float(target_feature_count_)) {
            float current_min_feature_size = extractor->min_size_multiplier_ * extractor->min_feature_size();
            float current_min_feature_diameter = sqrt(current_min_feature_size);
            float new_min_feature_diameter = current_min_feature_diameter - min_feature_diameter_step;
            float new_min_feature_size = new_min_feature_diameter * new_min_feature_diameter;
            if (new_min_feature_size < low_min_feature_size)
                new_min_feature_size = low_min_feature_size;
            min_size_multiplier = new_min_feature_size / extractor->min_feature_size();
        }

        // Increase the multiplier if too many features were detected last time
        if (float(nearby_feature_count) > trigger_scale * float(target_feature_count_)) {
            float current_min_feature_size = extractor->min_size_multiplier_ * extractor->min_feature_size();
            float current_min_feature_diameter = sqrt(current_min_feature_size);
            float new_min_feature_diameter = current_min_feature_diameter + min_feature_diameter_step;
            float new_min_feature_size = new_min_feature_diameter * new_min_feature_diameter;
            if (new_min_feature_size < low_min_feature_size)
                new_min_feature_size = low_min_feature_size;
            min_size_multiplier = new_min_feature_size / extractor->min_feature_size();
        }

        extractor->min_size_multiplier_ = min_size_multiplier;
        return;
    }

    // (iii) If there's no other frame data initialise the multiplier
    double low_min_feature_width(sqrt(double(low_min_feature_size)));
    double high_min_feature_width(sqrt(double(high_min_feature_size)));

    int const target_feature_count = target_feature_count_;
    std::function<int(double)> feature_count_from_min_feature_width = [&img, &mask, &extractor, target_feature_count](double min_feature_width) {
        std::vector<cv::KeyPoint> keypts;
        cv::Mat descriptors;
        double final_min_feature_size(min_feature_width * min_feature_width);
        float min_feature_size_multiplier = final_min_feature_size / extractor->min_feature_size();
        extractor->min_size_multiplier_ = min_feature_size_multiplier;
        extractor->extract(img, mask, keypts, descriptors);
        return (int)keypts.size() - target_feature_count;
    };

    double best_feature_width = bisection(low_min_feature_width, high_min_feature_width, feature_count_from_min_feature_width, 1.0);

    double best_min_feature_size(best_feature_width * best_feature_width);
    float min_feature_size_multiplier = best_min_feature_size / extractor->min_feature_size();
    extractor->min_size_multiplier_ = min_feature_size_multiplier;

    //Test: calculate min_feature_size vs feature count for the frame and record in an html graph - should be commented!!
    //save_graph_min_feature_size_vs_feature_count(img, mask, extractor, target_feature_count_);
}

void orb_feature_monitor::record_extraction_result(unsigned int feature_count, stella_vslam::feature::orb_extractor* extractor)
{    
    if (!extractor)
        return;

    // Store in the general metrics
    metrics::submit_frame_param(metrics::instance()->detected_feature_count, feature_count);
    metrics::submit_frame_param(metrics::instance()->min_feature_size, extractor->min_feature_size() * extractor->min_size_multiplier_);

    if (!use_feature_monitor())
        return;

    // Get the current frame and stage from the metrics object
    std::optional<stage_and_frame> stage_with_frame = metrics::instance()->timestamp_to_frame(metrics::instance()->current_frame_timestamp);

    // Store the result
    if (stage_with_frame) {
        frame_data data;
        data.frame = stage_with_frame->frame;
        data.multiplier = extractor->min_size_multiplier_;
        data.feature_count = feature_count;

        data_by_frame_[data.frame] = data;
        last_frame_data_ = data;
    }
    else
        last_frame_data_ = std::nullopt;

}

unsigned int orb_feature_monitor::default_min_feature_size_for_video_size(int width, int height) {

    unsigned int sd_min_feature_size(800); // min_feature_size for SD video
    unsigned int min_feature_size = stella_vslam_bfx::min_feature_size_from_invariant_min_feature_size(sd_min_feature_size, width * height);
    return min_feature_size;
}

    //orb_feature_monitor::orb_feature_monitor(std::shared_ptr<stella_vslam::config> cfg)
    //{
    //    // Set the min_pixel_size based on video size
    //    unsigned& min_feature_size = const_cast<unsigned&>(cfg->settings_.min_feature_size_);
    //    unsigned  invariant_min_feature_size = min_feature_size;
    //    min_feature_size = stella_vslam_bfx::min_feature_size_from_invariant_min_feature_size(invariant_min_feature_size, cfg->settings_.cols_ * cfg->settings_.rows_);
    //    spdlog::info("orb_feature_monitor: cols {}, rows {}, inv_min_feature_size {}, min_feature_size {}", cfg->settings_.cols_, cfg->settings_.rows_, invariant_min_feature_size, min_feature_size);
    //}

} // namespace stella_vslam_bfx
