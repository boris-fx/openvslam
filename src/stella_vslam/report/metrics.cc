#include "metrics.h"

#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <nlohmann/json.hpp>

#include "plot_html.h"

#include "stella_vslam/solve/fundamental_consistency.h"
#include <stella_vslam/data/frame.h>
namespace nlohmann {

template<class T>
nlohmann::json optional_to_json(const std::optional<T>& v) {
    if (v.has_value())
        return *v;
    else
        return nullptr;
}

template<class T>
std::optional<T> optional_from_json(const nlohmann::json& j) {
    if (j.is_null())
        return std::nullopt;
    else
        return j.get<T>();
}

} // namespace nlohmann

namespace stella_vslam_bfx {

    nlohmann::json video_metadata::to_json() const {

        return {
            {"name", name},
            {"filename", filename},
            {"gDriveID", gDriveID},

            {"video_width", video_width},
            {"video_height", video_height},
            {"start_frame", start_frame},
            {"end_frame", end_frame},
            {"pixel_aspect_ratio_used", pixel_aspect_ratio_used},

            {"groundTruthFocalLengthXPixels", nlohmann::optional_to_json(groundTruthFocalLengthXPixels)},
            {"groundTruthFilmBackWidthMM", nlohmann::optional_to_json(groundTruthFilmBackWidthMM)},
            {"groundTruthFilmBackHeightMM", nlohmann::optional_to_json(groundTruthFilmBackHeightMM)},
            {"groundTruthFocalLengthMM", nlohmann::optional_to_json(groundTruthFocalLengthMM)}
        };
    }

    bool video_metadata::from_json(const nlohmann::json& json) {

        name = json.at("name").get<std::string>();
        filename = json.at("filename").get<std::string>();
        gDriveID = json.at("gDriveID").get<std::string>();

        video_width = json.at("video_width").get<int>();
        video_height = json.at("video_height").get<int>();
        start_frame = json.at("start_frame").get<int>();
        end_frame = json.at("end_frame").get<int>();
        pixel_aspect_ratio_used = json.at("pixel_aspect_ratio_used").get<double>();

        groundTruthFocalLengthXPixels = nlohmann::optional_from_json<double>(json.at("groundTruthFocalLengthXPixels"));
        groundTruthFilmBackWidthMM = nlohmann::optional_from_json<double>(json.at("groundTruthFilmBackWidthMM"));
        groundTruthFilmBackHeightMM = nlohmann::optional_from_json<double>(json.at("groundTruthFilmBackHeightMM"));
        groundTruthFocalLengthMM = nlohmann::optional_from_json<double>(json.at("groundTruthFocalLengthMM"));

        return true;
    }

    std::optional<double> video_metadata::ground_truth_pixel_aspect_ratio() const
    {
        if (groundTruthFilmBackWidthMM && groundTruthFilmBackHeightMM) {
            double film_back_aspect_ratio = groundTruthFilmBackWidthMM.value() / groundTruthFilmBackHeightMM.value();
            double image_aspect_ratio = double(video_width) / double(video_height);
            return film_back_aspect_ratio / image_aspect_ratio;

        }
        return {};
    }

    std::optional<double> video_metadata::ground_truth_focal_length_x_pixels() const {
        if (groundTruthFocalLengthXPixels)
            return groundTruthFocalLengthXPixels.value();
        if (groundTruthFilmBackWidthMM && groundTruthFocalLengthMM)
            return groundTruthFocalLengthMM.value() * video_width / groundTruthFilmBackWidthMM.value();
        return {};
    }

    std::optional<double> video_metadata::calculated_focal_length_mm(double calculated_focal_length_x_pixels) const
    {
        if (groundTruthFilmBackWidthMM)
            return calculated_focal_length_x_pixels * groundTruthFilmBackWidthMM.value() / video_width;
        return {};
    }

    //////////////////////////////////////////////////////////////

    debugging_setup::debugging_setup()
        : debug_initialisation(false)
    {
    }

    nlohmann::json debugging_setup::to_json() const {
        return {
            {"debug_initialisation", debug_initialisation}
        };
    }

    bool debugging_setup::from_json(const nlohmann::json& json) {
        debug_initialisation = json.at("debug_initialisation").get<bool>();

        return true;
    }
    //////////////////////////////////////////////////////////////

    nlohmann::json timings::to_json() const {
        return {
            {"forward_mapping", forward_mapping},
            {"backward_mapping", backward_mapping},
            {"loop_closing", loop_closing},
            {"optimisation", optimisation},
            {"tracking", tracking}
        };
    }

    bool timings::from_json(const nlohmann::json& json) {
        forward_mapping = json.at("forward_mapping").get<double>();
        backward_mapping = json.at("backward_mapping").get<double>();
        loop_closing = json.at("loop_closing").get<double>();
        optimisation = json.at("optimisation").get<double>();
        tracking = json.at("tracking").get<double>();

        return true;
    }

    double timings::total_time_sec() const {
        return  forward_mapping + backward_mapping + loop_closing + optimisation + tracking;
    }

    //////////////////////////////////////////////////////////////

    void to_json(nlohmann::json& j, const stage_and_frame& e)
    {
        j = { {"stage", e.stage}, {"frame", e.frame} };
    }

    void from_json(const nlohmann::json& j, stage_and_frame& e) {
        j.at("stage").get_to(e.stage);
        j.at("frame").get_to(e.frame);
    }

    void to_json(nlohmann::json& j, const metrics::focal_estimate& e)
    {
        j = { {"estimate", e.estimate}, {"type", e.type}, {"stage_with_frame", e.stage_with_frame} };
    }

    void from_json(const nlohmann::json& j, metrics::focal_estimate& e) {
        j.at("estimate").get_to(e.estimate);
        j.at("type").get_to(e.type);
        j.at("stage_with_frame").get_to(e.stage_with_frame);
    }

    template<typename T>
    void to_json(nlohmann::json& j, const metrics::stage_and_frame_param<T>& e)
    {
        j = { {"by_stage_and_frame", e.by_stage_and_frame} };
    }

    template<typename T>
    void from_json(const nlohmann::json& j, metrics::stage_and_frame_param<T>& e) {
        j.at("by_stage_and_frame").get_to(e.by_stage_and_frame);
    }

    template<typename T>
    std::list<curve_section> metrics::stage_and_frame_param<T>::graph() const
    {
        std::list<curve_section> gg;
        for (int stage = 0; stage < max_stage; ++stage) {
            std::map<double, double> g;
            for (auto const& i : by_stage_and_frame[stage])
                g[i.first] = i.second;
            if (!g.empty())
                gg.push_back({ g, stage });
        }
        return gg;
    }


    template<typename T>
    void to_json(nlohmann::json& j, const metrics::stage_and_frame_pair_param<T>& e)
    {
        j = { {"by_stage_and_frame", e.by_stage_and_frame} };
    }

    template<typename T>
    void from_json(const nlohmann::json& j, metrics::stage_and_frame_pair_param<T>& e) {
        j.at("by_stage_and_frame").get_to(e.by_stage_and_frame);
    }

    template<typename T>
    std::list<curve_section> metrics::stage_and_frame_pair_param<T>::graph() const
    {
        std::list<curve_section> gg;
        for (int stage = 0; stage < max_stage; ++stage) {
            std::map<double, double> g;
            for (auto const& i : by_stage_and_frame[stage])
                g[i.first.second] = i.second;
            if (!g.empty())
                gg.push_back({ g, stage });
        }
        return gg;
    }

    template<typename T>
    std::list<curve_section> metrics::stage_and_frame_pair_param<T>::frame_separation_graph() const
    {
        std::list<curve_section> gg;
        for (int stage = 0; stage < max_stage; ++stage) {
            std::map<double, double> g;
            for (auto const& i : by_stage_and_frame[stage])
                g[i.first.second] = i.first.second - i.first.first;
            if (!g.empty())
                gg.push_back({ g, stage });
        }
        return gg;
    }

    //////////////////////////////////////////////////////////////

    const double metrics::par_percent_error_trigger = 20.0;
    const double metrics::focal_percent_error_trigger = 50.0;

    metrics* metrics::get_instance() {
        if (!m_instance)
            m_instance = new metrics();
        return m_instance;
    }

    metrics* metrics::instance() {
        if (!m_instance)
            m_instance = new metrics();
        return m_instance;
    }

    void metrics::clear() {
        if (m_instance) {
            delete m_instance;
            m_instance = nullptr;
        }
    }

    initialisation_debugging& metrics::initialisation_debug() {
        metrics* m = get_instance();
        m->initialisation_debug_object.is_active = m->debugging.debug_initialisation;
        return m->initialisation_debug_object;
    }

    nlohmann::json metrics::to_json() const {
        return {
            {"debugging", debugging.to_json()},
            {"input_video_metadata", input_video_metadata.to_json()},
            {"track_timings", track_timings.to_json()},
            {"calculated_focal_length_x_pixels", calculated_focal_length_x_pixels},
            {"solved_frame_count", solved_frame_count},
            {"unsolved_frame_count", unsolved_frame_count},
            {"num_points", num_points},
            {"initialisation_frames", initialisation_frames},
            {"intermediate_focal_estimates", intermediate_focal_estimates},
            {"map_size", map_size},
            {"tracking_fail_count", tracking_fail_count},
            {"mapping_reset_timestamps", mapping_reset_timestamps},
            {"mapping_reset_frames", mapping_reset_frames}

        };
    }

    bool metrics::from_json(const nlohmann::json& json) {

        debugging.from_json(json.at("debugging"));
        input_video_metadata.from_json(json.at("input_video_metadata"));
        track_timings.from_json(json.at("track_timings"));
        calculated_focal_length_x_pixels = json.at("calculated_focal_length_x_pixels").get<double>();
        solved_frame_count = json.at("solved_frame_count").get<int>();
        unsolved_frame_count = json.at("unsolved_frame_count").get<int>();
        num_points = json.at("num_points").get<int>();
        initialisation_frames = json.at("initialisation_frames").get<std::set<std::set<int>>>();
        intermediate_focal_estimates = json.at("intermediate_focal_estimates").get<std::list<focal_estimate>>();
        map_size = json.at("map_size").get<stage_and_frame_param<unsigned int>>();
        tracking_fail_count = json.at("tracking_fail_count").get<stage_and_frame_param<unsigned int>>();
        mapping_reset_timestamps = json.at("mapping_reset_timestamps").get<std::list<double>>();
        mapping_reset_frames = json.at("mapping_reset_frames").get<std::list<int>>();

        return true;
    }

    std::optional<stage_and_frame> metrics::timestamp_to_frame(double timestamp)
    {
        if (!timestamp_to_stage_and_frame)
            return std::nullopt;
        auto f = timestamp_to_stage_and_frame->find(timestamp);
        if (f != timestamp_to_stage_and_frame->end())
            return f->second;
        return std::nullopt;
    }

    void metrics::submit_intermediate_focal_estimate(focal_estimation_type type, double estimate)
    {
        std::optional<stage_and_frame> stage_with_frame = timestamp_to_frame(current_frame_timestamp);
        if (!stage_with_frame)
            return;

        intermediate_focal_estimates.push_back({ estimate, type, stage_with_frame.value() });
    }

    void metrics::submit_map_size_and_tracking_fails(double timestamp, unsigned int map_keyframe_count, unsigned int tracking_fails)
    {
        std::optional<stage_and_frame> stage_with_frame = timestamp_to_frame(timestamp);
        if (!stage_with_frame)
            return;
        
        map_size.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = map_keyframe_count;
        tracking_fail_count.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = tracking_fails;

        //map_size.by_timestamp[timestamp] = map_keyframe_count;
        //tracking_fail_count.by_timestamp[timestamp] = tracking_fails;
    }

    void metrics::submit_area_matching(int frame_1_points, int fail_prematched, int fail_scale, int fail_cell, int fail_hamming, int fail_ratio,
                                       int count_indices_exist, int count_num_indices, int num_matches)
    {
        if (!capture_area_matching)
            return;

        std::optional<stage_and_frame> stage_with_frame_0 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[0]);
        if (!stage_with_frame_0)
            return;
        std::optional<stage_and_frame> stage_with_frame_1 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[1]);
        if (!stage_with_frame_1)
            return;
        int stage = stage_with_frame_0.value().stage;
        std::pair<int, int> frames = { stage_with_frame_0.value().frame, stage_with_frame_1.value().frame };

        this->area_match_frame_1_points.by_stage_and_frame[stage][frames] = frame_1_points;
        this->area_match_fail_prematched.by_stage_and_frame[stage][frames] = fail_prematched;
        this->area_match_fail_scale.by_stage_and_frame[stage][frames] = fail_scale;
        this->area_match_fail_cell.by_stage_and_frame[stage][frames] = fail_cell;
        this->area_match_fail_hamming.by_stage_and_frame[stage][frames] = fail_hamming;
        this->area_match_fail_ratio.by_stage_and_frame[stage][frames] = fail_ratio;
        this->area_match_num_matches.by_stage_and_frame[stage][frames] = num_matches;
        this->area_match_ave_candidates.by_stage_and_frame[stage][frames] = count_indices_exist>0 ? double(count_num_indices)/double(count_indices_exist) : 0.0;
        this->area_match_num_attempted.by_stage_and_frame[stage][frames] = frame_1_points - fail_prematched - fail_scale;
    }

    void metrics::submit_triangulation_debugging( std::optional<std::pair<double, double>> in_num_triangulated_points,
                                                  std::optional<std::pair<double, double>> in_num_valid_triangulated_points,
                                                  std::optional<std::pair<double, double>> in_triangulation_parallax,
                                                  std::optional<std::pair<double, double>> in_triangulation_ambiguity )
    {
        std::optional<stage_and_frame> stage_with_frame_0 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[0]);
        if (!stage_with_frame_0)
            return;
        std::optional<stage_and_frame> stage_with_frame_1 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[1]);
        if (!stage_with_frame_1)
            return;

        int stage = stage_with_frame_0.value().stage;
        std::pair<int, int> frames = { stage_with_frame_0.value().frame, stage_with_frame_1.value().frame };
        if (in_num_triangulated_points) {
            num_triangulated_points.by_stage_and_frame[stage][frames] = in_num_triangulated_points.value().first;
            if (!min_num_triangulated_points)
                min_num_triangulated_points = in_num_triangulated_points.value().second;
        }
        if (in_num_valid_triangulated_points) {
            num_valid_triangulated_points.by_stage_and_frame[stage][frames] = in_num_valid_triangulated_points.value().first;
            if (!min_num_valid_triangulated_points)
                min_num_valid_triangulated_points = in_num_valid_triangulated_points.value().second;
        }
        if (in_triangulation_parallax) {
            // NB: raw value is cosine of the parallax
            const double pi = 3.14159265358979323846;
            double parallax_degrees = std::acos(in_triangulation_parallax.value().first) * 180.0 / pi;
            triangulation_parallax.by_stage_and_frame[stage][frames] = parallax_degrees;
            if (!max_triangulation_parallax) {
                double parallax_threshold_degrees = std::acos(in_triangulation_parallax.value().second) * 180.0 / pi;
                max_triangulation_parallax = parallax_threshold_degrees;
            }
        }
        if (in_triangulation_ambiguity) {
            triangulation_ambiguity.by_stage_and_frame[stage][frames] = in_triangulation_ambiguity.value().first;
            if (!max_triangulation_ambiguity)
                max_triangulation_ambiguity = in_triangulation_ambiguity.value().second;
        }
    }

    void metrics::submit_mapping_reset(double timestamp)
    {
        mapping_reset_timestamps.push_back(timestamp);
    }

    void metrics::submit_initialiser_constrained_matching_stats(std::vector<bool> const& homography_inliers, std::vector<bool> const& fundamental_inliers)
    {
        std::optional<stage_and_frame> stage_with_frame_0 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[0]);
        if (!stage_with_frame_0)
            return;
        std::optional<stage_and_frame> stage_with_frame_1 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[1]);
        if (!stage_with_frame_1)
            return;
        int stage = stage_with_frame_0.value().stage;
        std::pair<int, int> frames = { stage_with_frame_0.value().frame, stage_with_frame_1.value().frame };

        if (homography_inliers.size() == fundamental_inliers.size())
            initialisation_homography_fundamental_candidates.by_stage_and_frame[stage][frames] = homography_inliers.size();
        initialisation_homography_inliers.by_stage_and_frame[stage][frames] = std::count(homography_inliers.begin(), homography_inliers.end(), true);
        initialisation_fundamental_inliers.by_stage_and_frame[stage][frames] = std::count(fundamental_inliers.begin(), fundamental_inliers.end(), true);
    
        // NB: Thresholds are min_set_size=8, in fundamental_solver::find_via_ransac()
        //                and min_set_size=8, in homography_solver::find_via_ransac()
    
    }

    void metrics::submit_focal_length_estimate(double focal_length, double stability)
    {
        std::optional<stage_and_frame> stage_with_frame_0 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[0]);
        if (!stage_with_frame_0)
            return;
        std::optional<stage_and_frame> stage_with_frame_1 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[1]);
        if (!stage_with_frame_1)
            return;
        int stage = stage_with_frame_0.value().stage;
        std::pair<int, int> frames = { stage_with_frame_0.value().frame, stage_with_frame_1.value().frame };

        initialisation_focal_length_estimate.by_stage_and_frame[stage][frames] = focal_length;
        initialisation_focal_length_stability.by_stage_and_frame[stage][frames] = stability;
    }

    void metrics::submit_2_3_view_focal_length(std::optional<double> focal_length_2_view,
                                               std::optional<double> focal_length_3_view)
    {
        std::optional<stage_and_frame> stage_with_frame_0 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[0]);
        if (!stage_with_frame_0)
            return;
        std::optional<stage_and_frame> stage_with_frame_1 = timestamp_to_frame(initialisation_debug_object.current_init_frame_timestamps[1]);
        if (!stage_with_frame_1)
            return;

        int stage = stage_with_frame_0.value().stage;
        std::pair<int, int> frames = { stage_with_frame_0.value().frame, stage_with_frame_1.value().frame };
        if (focal_length_2_view)
            initialisation_focal_length_estimate_2_view.by_stage_and_frame[stage][frames] = focal_length_2_view.value();
        if (focal_length_3_view)
            initialisation_focal_length_estimate_3_view.by_stage_and_frame[stage][frames] = focal_length_3_view.value();

        auto a = initialisation_focal_length_estimate_2_view.graph();
        int yy = 0;
    }
#if 0
    template<typename Tp, typename T>
    void metrics::submit_frame_param(stage_and_frame_param_with_threshold<Tp> &param, T value, T threshold) {
        std::optional<stage_and_frame> stage_with_frame = instance()->timestamp_to_frame(instance()->current_frame_timestamp);
        if (!stage_with_frame)
            return;
        param.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = (Tp)value;
        param.threshold = (Tp)threshold;
    }
    template void metrics::submit_frame_param<int, int>(stage_and_frame_param_with_threshold<int> &param, int value, int threshold);
    template void metrics::submit_frame_param<int, unsigned int>(stage_and_frame_param_with_threshold<int> &param, unsigned int value, unsigned int threshold);
    template void metrics::submit_frame_param<unsigned int, int>(stage_and_frame_param_with_threshold<unsigned int>& param, int value, int threshold);
    template void metrics::submit_frame_param<unsigned int, unsigned int>(stage_and_frame_param_with_threshold<unsigned int> &param, unsigned int value, unsigned int threshold);
    template void metrics::submit_frame_param<double, double>(stage_and_frame_param_with_threshold<double> &param, double value, double threshold);

    template<typename Tp, typename T>
    void metrics::submit_frame_param(stage_and_frame_param<Tp>& param, T value) {
        std::optional<stage_and_frame> stage_with_frame = instance()->timestamp_to_frame(instance()->current_frame_timestamp);
        if (!stage_with_frame)
            return;
        param.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = (Tp)value;
    }
    template void metrics::submit_frame_param<int, int>(stage_and_frame_param<int>& param, int value);
    template void metrics::submit_frame_param<int, unsigned int>(stage_and_frame_param<int>& param, unsigned int value);
    template void metrics::submit_frame_param<unsigned int, int>(stage_and_frame_param<unsigned int>& param, int value);
    template void metrics::submit_frame_param<unsigned int, unsigned int>(stage_and_frame_param<unsigned int>& param, unsigned int value);
    template void metrics::submit_frame_param<double, double>(stage_and_frame_param<double>& param, double value);
#endif
    //void metrics::submit_tracking_by_motion_matches_1(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_motion_matches_1);
    //}
    //void metrics::submit_tracking_by_motion_matches_2(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_motion_matches_2);
    //}
    //void metrics::submit_tracking_by_motion_matches_optimised(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_motion_matches_optimised);
    //}
    //void metrics::submit_tracking_by_bow_matches(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_bow_matches);
    //}
    //void metrics::submit_tracking_by_bow_matches_optimised(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_bow_matches_optimised);
    //}
    //void metrics::submit_tracking_by_robust_matches(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_robust_matches);
    //}
    //void metrics::submit_tracking_by_robust_matches_optimised(int num_matches, int min_num_matches_threshold) {
    //    submit_frame_param(num_matches, min_num_matches_threshold, tracking_robust_matches_optimised);
    //}

    /*template<typename T>
    void transform_metrics_frame_data(metrics::stage_and_frame_param<T> &data,
                                      std::map<double, stage_and_frame> const& index_transform)
    {
        data.by_frame.clear();
        for (auto const& i : data.by_timestamp)
        {
            auto f = index_transform.find(i.first);
            if (f != index_transform.end())
                data.by_frame[f->second.frame] = i.second;
        }
    }*/

    void metrics::create_frame_metrics() {
        initialisation_frames.clear();
        for (auto const& initialisation_attempt_timestamps : initialisation_frame_timestamps) {
            std::set<int> initialisation_attempt_frames;
            for (auto const& timestamp : initialisation_attempt_timestamps) {
                std::optional<stage_and_frame> stage_with_frame = timestamp_to_frame(timestamp);
                if (stage_with_frame)
                    initialisation_attempt_frames.insert(stage_with_frame.value().frame);
            }
            initialisation_frames.insert(initialisation_attempt_frames);
        }

        initialisation_debug_object.create_frame_data(*timestamp_to_stage_and_frame);

        //for (auto& estimate : intermediate_focal_estimates)
        //    if (auto frame = timestamp_to_frame(estimate.timestamp))
        //        estimate.frame = frame.value().frame;

        mapping_reset_frames.clear();
        for (auto const& timestamp : mapping_reset_timestamps)
            if (auto frame = timestamp_to_frame(timestamp))
                mapping_reset_frames.push_back(frame.value().frame);

        //transform_metrics_frame_data(map_size);
        //transform_metrics_frame_data(tracking_fail_count, *timestamp_to_stage_and_frame);
    }

    double percent_difference(double a, double b)
    {
        if (a < b)
            return (100.0 * (b - a) / a);
        else
            return (100.0 * (a - b) / b);
    }

    bool within_percent(double a, double b, double percent) {
        return percent_difference(a, b) < percent;
    }

    bool within_percent(std::optional<double> a, double b, double percent) {
        if (!a.has_value())
            return true;
        return within_percent(a.value(), b, percent);
    }

    bool within_percent(double a, std::optional<double> b, double percent) {
        return within_percent(b, a, percent);
    }

    std::set<std::pair<std::string, tracking_problem_level>> metrics::problems() const
    {
        std::set<std::pair<std::string, tracking_problem_level>> problem_set;
        if (unsolved_frame_count != 0)
            if (solved_frame_count == 0)
                problem_set.insert({ "No tracked frames", tracking_problem_fatal });
            else
                problem_set.insert({ std::to_string(unsolved_frame_count) + " untracked frames", tracking_problem_warning });
        if (!within_percent(input_video_metadata.ground_truth_focal_length_x_pixels(), calculated_focal_length_x_pixels, focal_percent_error_trigger))
            problem_set.insert({ "Wrong focal length", tracking_problem_warning });
        if (!within_percent(input_video_metadata.pixel_aspect_ratio_used, input_video_metadata.ground_truth_pixel_aspect_ratio(), par_percent_error_trigger))
            problem_set.insert({ "Wrong par", tracking_problem_warning });
        return problem_set;
    }

    std::optional<tracking_problem_level> metrics::max_problem_level() const
    {
        bool have_warning(false), have_fatal(false);
        auto probs = problems();
        for (auto const& problem : probs) {
            if (problem.second == tracking_problem_warning)
                have_warning = true;
            if (problem.second == tracking_problem_fatal)
                have_fatal = true;
        }
        if (have_fatal)
            return tracking_problem_fatal;
        if (have_warning)
            return tracking_problem_warning;
        return {};
    }

    template<typename T>
    std::string to_string(std::set<T> const& s)
    {
        std::string str;
        int n(s.size() - 1), i(0);
        for (auto const& item : s) {
            str += std::to_string(item);
            if (i != n)
                str += ", ";
            ++i;
        }
        return str;
    }

    std::string print_initialisation(std::set<int> const& frames, std::list<metrics::focal_estimate> const& intermediate_focal_estimates)
    {
        if (frames.empty())
            return "";
        auto f = std::find_if(intermediate_focal_estimates.begin(), intermediate_focal_estimates.end(),
            [&](const metrics::focal_estimate& e) { return e.type == focal_estimation_type::initialisation_before_ba && e.stage_with_frame.frame==*(frames.rbegin()); });
        if (f != intermediate_focal_estimates.end())
            return "{" + to_string(frames) + "} Focal length: " + std::to_string(f->estimate);
        return "{" + to_string(frames) + "}";
    }

    std::vector<std::string> splitString(const std::string& str)
    {
        std::vector<std::string> tokens;

        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, '\n')) {
            tokens.push_back(token);
        }

        return tokens;
    }

    std::array< std::map<double, double>, 2> split_graph_by_frame(std::map<double, double> const& graph, double frame)
    {
        std::array<std::map<double, double>, 2> result;
        for (auto const& point : graph)
            result[point.first < frame ? 0 : 1][point.first] = point.second;
        return result;
    }
#if 0
    // Split a graph at the largest frame in a set
    std::set<std::map<double, double>> split_graph_by_frame_set(std::map<double, double> const& graph, std::set<std::set<int>> const& frames)
    {
        std::set<double> split_points;
        for (auto const& frame_set : frames)
            if (!frame_set.empty())
                split_points.insert(0.5 + *frame_set.rbegin());

        if (split_points.empty())
            return { graph };

        for (auto const& split_point : split_points)
            spdlog::info("split_graph_by_frame_set: split point: {}", split_point);

        std::set<std::map<double, double>> split_graph;
        std::map<double, double> remaining_graph = graph;
        for (auto const& split_point : split_points) {
            std::array<std::map<double, double>, 2> graph_pair = split_graph_by_frame(remaining_graph, split_point);
            remaining_graph = graph_pair[1];
            split_graph.insert(graph_pair[0]);
        }
        split_graph.insert(remaining_graph);

        return split_graph;
    }
#endif

void spdlog_frame_map(std::string const& name, std::map<double, double> const& frame_map)
{
    std::stringstream ss;
    ss << name << " - (" << frame_map.size() << " frames) ";
    for (auto const& f : frame_map)
        ss << f.first << ", ";
    spdlog::info("{}", ss.str());
}

std::list<curve_section> split_graph_by_frames(std::string const& name, std::map<int, curve_section> const& graphs, std::list<int> const& frames)
{
    std::set<double> split_points;
    for (auto const& frame : frames)
        split_points.insert(0.5 + frame);

    if (split_points.empty()) { // convert the map to a list
        std::list<curve_section> result;
        std::transform(graphs.begin(), graphs.end(), back_inserter(result), [](const auto& val) {return val.second; });
        return result;
    }

    (void)name;
    //for (auto const& split_point : split_points)
    //    spdlog::info("split_graph_by_frames: {} split point: {}", name, split_point);
    std::list<curve_section> split_graph;
    for (auto const& graph : graphs) {
            
        std::map<double, double> remaining_graph = graph.second;
        for (auto const& split_point : split_points) {
            std::array<std::map<double, double>, 2> graph_pair = split_graph_by_frame(remaining_graph, split_point);
            remaining_graph = graph_pair[1];
            split_graph.push_back({ graph_pair[0], graph.second.stage });
        }
        split_graph.push_back({ remaining_graph, graph.second.stage });
    }
    //spdlog_frame_map("Unsplit graph", graph);
    //for (auto const& s : split_graph)
    //    spdlog_frame_map("Split graph", s);

    return split_graph;
}

double percentile_y_value(std::array<std::map<int, curve_section>, 4> const& graphs, double percentile)
{

    std::vector<double> ys;
    for (int i = 0; i < 4; ++i)
        for (auto const& section : graphs[i])
            for (auto const& xy : section.second)
                ys.push_back(xy.second);
    return stella_vslam_bfx::quantile(ys, {percentile})[0];
}

double percentile_y_value(curve_section const& graph, double percentile)
{
    std::array<std::map<int, curve_section>, 4>  graphs;
    graphs[0][0] = graph;
    return percentile_y_value(graphs, percentile);
}

struct graph_stats {
    graph_stats(std::map<double, double> const& graph)
    : mean(0), min(0), max(0)
    {
        if (!graph.empty()) {
            double sum(0);
            for (auto const& value : graph) {
                if (min > value.second)
                    min = value.second;
                if (max < value.second)
                    max = value.second;
                sum += value.second;
            }
            mean = sum / double(graph.size());
        }
    }
    graph_stats(std::list<curve_section> const& curves)
        : mean(0), min(0), max(0)
    {
        double sum(0);
        int count(0);
        for (auto const& curve : curves) {
            for (auto const& value : curve) {
                if (min > value.second)
                    min = value.second;
                if (max < value.second)
                    max = value.second;
                sum += value.second;
                ++count;
            }
        }
        if (count>0)
            mean = sum / double(count);
    }
    double mean;
    double min;
    double max;
};

template<typename T_V, typename T_T>
std::string noncritical_style_if_less(std::optional<T_V> value, T_T threshold)
{
    if (!value || (value.value() >= threshold))
        return "";
    return " class = \"outside_threshold_noncritical\"";
}

template<typename T_V, typename T_T>
std::string style_if_less(std::optional<T_V> value, std::optional<T_T> threshold)
{
    if (!value || !threshold || (value.value() >= threshold.value()))
        return "";
    return " class = \"outside_threshold\"";
}

template<typename T_V, typename T_T>
std::string style_if_less(std::optional<T_V> value, T_T threshold)
{
    return style_if_less(value, std::optional<T_T>(threshold));
}

template<typename T_V, typename T_T>
std::string style_if_less(T_V value, T_T threshold)
{
    return style_if_less(std::optional<T_V>(value), std::optional<T_T>(threshold));
}


template<typename T_V, typename T_T>
std::string style_if_greater(std::optional<T_V> value, std::optional<T_T> threshold)
{
    if (!value || !threshold || value.value() <= threshold)
        return "";
    return " class = \"outside_threshold\"";
}

template<typename T>
std::string table_value(std::optional<T> value)
{
    if (!value)
        return "";
    return std::to_string(value.value());
}

void add_graph_style(std::stringstream& html)
{

    html << ".graph {\n";
    html << "	margin-bottom:1em;\n";
    html << "  font:normal 100%/150% arial,helvetica,sans-serif;\n";
    html << "}\n";

    html << ".graph caption {\n";
    html << "	font:bold 150%/120% arial,helvetica,sans-serif;\n";
    html << "	padding-bottom:0.33em;\n";
    html << "}\n";

    html << ".graph tbody th {\n";
    html << "	text-align:right;\n";
    html << "}\n";

    html << "@supports (display:grid) {\n";

    html << "	@media (min-width:32em) {\n";

    html << "		.graph {\n";
    html << "			display:block;\n";
    html << "      width:600px;\n";
    html << "      height:300px;\n";
    html << "		}\n";

    html << "		.graph caption {\n";
    html << "			display:block;\n";
    html << "		}\n";

    html << "		.graph thead {\n";
    html << "			display:none;\n";
    html << "		}\n";

    html << "		.graph tbody {\n";
    html << "			position:relative;\n";
    html << "			display:grid;\n";
    html << "			grid-template-columns:repeat(auto-fit, minmax(2em, 1fr));\n";
    html << "			column-gap:2.5%;\n";
    html << "			align-items:end;\n";
    html << "			height:100%;\n";
    html << "			margin:3em 0 1em 2.8em;\n";
    html << "			padding:0 1em;\n";
    html << "			border-bottom:2px solid rgba(0,0,0,0.5);\n";
    html << "			background:repeating-linear-gradient(\n";
    html << "				180deg,\n";
    html << "				rgba(170,170,170,0.7) 0,\n";
    html << "				rgba(170,170,170,0.7) 1px,\n";
    html << "				transparent 1px,\n";
    html << "				transparent 20%\n";
    html << "			);\n";
    html << "		}\n";

    html << "		.graph tbody:before,\n";
    html << "		.graph tbody:after {\n";
    html << "			position:absolute;\n";
    html << "			left:-3.2em;\n";
    html << "			width:2.8em;\n";
    html << "			text-align:right;\n";
    html << "			font:bold 80%/120% arial,helvetica,sans-serif;\n";
    html << "		}\n";

    html << "		.graph tbody:before {\n";
    html << "			content:\" \";\n"; // Axis text max
    html << "			top:-0.6em;\n";
    html << "		}\n";

    html << "		.graph tbody:after {\n";
    html << "			content:\"0 \";\n"; // Axis text min
    html << "			bottom:-0.6em;\n";
    html << "		}\n";

    html << "		.graph tr {\n";
    html << "			position:relative;\n";
    html << "			display:block;\n";
    html << "		}\n";

    html << "		.graph tr:hover {\n";
    html << "			z-index:999;\n";
    html << "		}\n";

    html << "		.graph th,\n";
    html << "		.graph td {\n";
    html << "			display:block;\n";
    html << "			text-align:center;\n";
    html << "		}\n";

    html << "		.graph tbody th {\n";
    html << "			position:absolute;\n";
    html << "			top:-3em;\n";
    html << "			left:0;\n";
    html << "			width:100%;\n";
    html << "			font-weight:normal;\n";
    html << "			text-align:center;\n";
    html << "     white-space:nowrap;\n";
    html << "			text-indent:0;\n";
    html << "			transform:rotate(-45deg);\n";
    html << "		}\n";

    html << "		.graph tbody th:after {\n";
    html << "			content:\"\";\n";
    html << "		}\n";

    html << "		.graph td {\n";
    html << "			width:100%;\n";
    html << "			height:100%;\n";
    html << "			background:#555;\n"; // Colour of the bars
    html << "			border-radius:0.5em 0.5em 0 0;\n";
    html << "			transition:background 0.5s;\n";
    html << "		}\n";

    html << "		.graph tr:hover td {\n";
    html << "			opacity:0.7;\n";
    html << "		}\n";

    html << "		.graph td span {\n";
    html << "			overflow:hidden;\n";
    html << "			position:absolute;\n";
    html << "			left:50%;\n";
    html << "			top:50%;\n";
    html << "			width:0;\n";
    html << "			padding:0.5em 0;\n";
    html << "			margin:-1em 0 0;\n";
    html << "			font:normal 85%/120% arial,helvetica,sans-serif;\n";
    html << "/* 			background:white; */\n";
    html << "/* 			box-shadow:0 0 0.25em rgba(0,0,0,0.6); */\n";
    html << "			font-weight:bold;\n";
    html << "			opacity:0;\n";
    html << "			transition:opacity 0.5s;\n";
    html << "      color:white;\n";
    html << "		}\n";

    html << "		.toggleGraph:checked + table td span,\n";
    html << "		.graph tr:hover td span {\n";
    html << "			width:4em;\n";
    html << "			margin-left:-2em; /* 1/2 the declared width */\n";
    html << "			opacity:1;\n";
    html << "		}\n";

    html << "	} /* min-width:32em */\n";

    html << "} /* grid only */\n";
}

void add_init_failure_mode_table(std::stringstream& html, std::list<std::pair<std::string, int>> data, std::string html_class)
{
    int max_data(1);
    for (auto const& val : data)
        if (max_data < val.second)
            max_data = val.second;

    if (!html_class.empty())
        html << "<table class=\"" << html_class << "\">\n";
    else
        html << "<table>\n";
    //html << "<caption>Initialisation Failure Modes</caption>\n";
    for (int i = 0; i < 5; ++i)
        html << "<br>\n";
    html << "<thead>\n";
    html << "	<tr>\n";
    html << "		<th scope=\"col\">Stage</th>\n";
    html << "		<th scope=\"col\">Fails</th>\n";
    html << "	</tr>\n";
    html << "</thead>\n";
    html << "<tbody>\n";

    for (auto const& val : data) {
        float percent = 100.0f * float(val.second) / float(max_data);
        html << "<tr style=\"height:" << percent << "%\">\n";
        html << "	<th scope=\"row\">" << val.first << "</th>\n";
        html << "	<td><span>" << val.second << "</span></td>\n";
        html << "</tr>\n";
    }

    html << "	</tbody>\n";
    html << "</table>\n";

    for (int i=0; i<5; ++i)
        html << "<br>\n";

}

// NB: a is not less than b if either is unkmown
template<typename TA, typename TB>
bool optional_less(std::optional<TA> const& a, std::optional<TB> const& b)
{
    if (!a || !b)
        return false;
    return a.value() < b.value();
}

// NB: a is not greater than b if either is unkmown
template<typename TA, typename TB>
bool optional_greater(std::optional<TA> const& a, std::optional<TB> const& b)
{
    if (!a || !b)
        return false;
    return a.value() > b.value();
}

void metrics::add_matching_details(std::stringstream& html, std::list<curve_section> const& graph_feature_count, std::list<curve_section> const& graph_num_matches) const
{
    struct initialisation_attempt {
        int frame_0;
        int frame_1;

        // Unguided matching
        int num_features_0;
        int num_features_1;
        int num_unguided_matches;

        // Guided matching
        std::optional<double> homography_fundamental_candidates;;
        std::optional<double> homography_inliers;
        std::optional<double> fundamental_inliers;

        // Focal length estimation
        std::optional<double> focal_length_estimate;
        std::optional<double> focal_length_stability;
        std::optional<double> focal_length_estimate_2_view;
        std::optional<double> focal_length_estimate_3_view;

        // Reconstruction
        std::optional<int>    num_valid_triangulated_points;
        std::optional<double> triangulation_ambiguity;
        std::optional<double> triangulation_parallax;
        std::optional<int>    num_triangulated_points;

    };
    std::map<std::array<int, 2>, initialisation_attempt> initialisation_attempts;
    for (auto const& attempt_unguided_matches : initialisation_debug_object.p_num_matches.by_frame) {
        initialisation_attempt& attempt(initialisation_attempts[attempt_unguided_matches.first]);
        attempt.frame_0 = attempt_unguided_matches.first[0];
        attempt.frame_1 = attempt_unguided_matches.first[1];

        // Unguided matching
        attempt.num_unguided_matches = attempt_unguided_matches.second;
        int stage(0);
        auto const& feature_count_map(metrics::instance()->detected_feature_count.by_stage_and_frame[stage]);
        auto f_feature_count_0 = feature_count_map.find(attempt.frame_0);
        if (f_feature_count_0 != feature_count_map.end())
            attempt.num_features_0 = f_feature_count_0->second;
        auto f_feature_count_1 = feature_count_map.find(attempt.frame_1);
        if (f_feature_count_1 != feature_count_map.end())
            attempt.num_features_1 = f_feature_count_1->second;

        // Guided matching
        auto f_homography_fundamental_candidate = initialisation_homography_fundamental_candidates.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_homography_fundamental_candidate != initialisation_homography_fundamental_candidates.by_stage_and_frame[0].end())
            attempt.homography_fundamental_candidates = f_homography_fundamental_candidate->second;

        auto f_homography_inliers = initialisation_homography_inliers.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_homography_inliers != initialisation_homography_inliers.by_stage_and_frame[0].end())
            attempt.homography_inliers = f_homography_inliers->second;

        auto f_fundamental_inliers = initialisation_fundamental_inliers.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_fundamental_inliers != initialisation_fundamental_inliers.by_stage_and_frame[0].end())
            attempt.fundamental_inliers = f_fundamental_inliers->second;

        // Focal length estimation
        auto f_focal_length_estimate = initialisation_focal_length_estimate.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_focal_length_estimate != initialisation_focal_length_estimate.by_stage_and_frame[0].end())
            attempt.focal_length_estimate = f_focal_length_estimate->second;

        auto f_focal_length_stability = initialisation_focal_length_stability.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_focal_length_stability != initialisation_focal_length_stability.by_stage_and_frame[0].end())
            attempt.focal_length_stability = f_focal_length_stability->second;

        auto f_focal_length_estimate_2_view = initialisation_focal_length_estimate_2_view.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_focal_length_estimate_2_view != initialisation_focal_length_estimate_2_view.by_stage_and_frame[0].end())
            attempt.focal_length_estimate_2_view = f_focal_length_estimate_2_view->second;

        auto f_focal_length_estimate_3_view = initialisation_focal_length_estimate_3_view.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_focal_length_estimate_3_view != initialisation_focal_length_estimate_3_view.by_stage_and_frame[0].end())
            attempt.focal_length_estimate_3_view = f_focal_length_estimate_3_view->second;

        // Reconstruction
        auto f_num_valid_triangulated_points = num_valid_triangulated_points.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_num_valid_triangulated_points != num_valid_triangulated_points.by_stage_and_frame[0].end())
            attempt.num_valid_triangulated_points = f_num_valid_triangulated_points->second;

        auto f_triangulation_ambiguity = triangulation_ambiguity.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_triangulation_ambiguity != triangulation_ambiguity.by_stage_and_frame[0].end())
            attempt.triangulation_ambiguity = f_triangulation_ambiguity->second;

        auto f_triangulation_parallax = triangulation_parallax.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_triangulation_parallax != triangulation_parallax.by_stage_and_frame[0].end())
            attempt.triangulation_parallax = f_triangulation_parallax->second;

        auto f_num_triangulated_points = num_triangulated_points.by_stage_and_frame[0].find({ attempt.frame_0 , attempt.frame_1 });
        if (f_num_triangulated_points != num_triangulated_points.by_stage_and_frame[0].end())
            attempt.num_triangulated_points = f_num_triangulated_points->second;

    }

    // Matching details
    graph_stats stats_feature_count(graph_feature_count);
    graph_stats stats_num_matches(graph_num_matches);
    html << "<h2> Matching Details</h2>\n";
    html << "<p>Features detected per frame - average: " << stats_feature_count.mean << ", min/max: " << stats_feature_count.min << "/" << stats_feature_count.max << "</p>\n";
    html << "<p>Unguided matches per frame - average: " << stats_num_matches.mean << ", min/max: " << stats_num_matches.min << "/" << stats_num_matches.max << ", threshold (min): " << settings.min_num_valid_pts_ << "</p>\n";


    // Failure reasons
    int fail_num_unguided_matches(0);
    int fail_fundamental_homography_inliers(0);
    int fail_focal_length_stability(0);
    int fail_num_valid_triangulated_points(0);
    int fail_triangulation_ambiguity(0);
    int fail_triangulation_parallax(0);
    int fail_num_triangulated_points(0);
    for (auto const& attempt_iter : initialisation_attempts) {
        initialisation_attempt const& attempt(attempt_iter.second);
        if (attempt.num_unguided_matches < (int)settings.min_num_valid_pts_) {
            ++fail_num_unguided_matches;
            continue;
        }
        if ((attempt.fundamental_inliers < 8) && (attempt.homography_inliers < 8)) {
            ++fail_fundamental_homography_inliers;
            continue;
        }
        if (attempt.focal_length_stability < 2) {
            ++fail_focal_length_stability;
            continue;
        }
        if (optional_less(attempt.num_valid_triangulated_points, min_num_valid_triangulated_points)) {
            ++fail_num_valid_triangulated_points;
            continue;
        }
        if (optional_greater(attempt.triangulation_ambiguity, max_triangulation_ambiguity)) {
            ++fail_triangulation_ambiguity;
            continue;
        }
        if (optional_less(attempt.triangulation_parallax, max_triangulation_parallax)) {
            ++fail_triangulation_parallax;
            continue;
        }
        if (optional_less(attempt.num_triangulated_points, min_num_triangulated_points)) {
            ++fail_num_triangulated_points;
            continue;
        }
    }

    //html << "<p>Fails unguided matching: " << fail_num_unguided_matches << "</p>\n";
    //html << "<p>Fails   guided matching: " << fail_fundamental_homography_inliers << "</p>\n";
    //html << "<p>Fails      focal length: " << fail_focal_length_stability << "</p>\n";
    //html << "<p>Fails     triangulation: " << fail_num_valid_triangulated_points << "</p>\n";
    //html << "<p>Fails         ambiguity: " << fail_triangulation_ambiguity << "</p>\n";
    //html << "<p>Fails        parallax A: " << fail_triangulation_parallax << "</p>\n";
    //html << "<p>Fails        parallax B: " << fail_num_triangulated_points << "</p>\n";

    add_init_failure_mode_table(html, { { "Unguided Matching", fail_num_unguided_matches },
                                        { "Uncalibrated Guided Matching" , fail_fundamental_homography_inliers },
                                        { "Focal Length" , fail_focal_length_stability },
                                        { "Calibrated Guided Matching" , fail_num_valid_triangulated_points },
                                        { "Ambiguity" , fail_triangulation_ambiguity },
                                        { "Parallax A" , fail_triangulation_parallax },
                                        { "Parallax B" , fail_num_triangulated_points } }, "graph");

    //html << "<p>Guided matches (uncalibrated) per frame - average: " << stats_num_matches.mean << ", min/max: " << stats_num_matches.min << "/" << stats_num_matches.max << ", threshold (min): " << settings.min_num_valid_pts_ << "</p>\n";
    //html << "<p>Guided matches (calibrated) per frame - average: " << stats_num_matches.mean << ", min/max: " << stats_num_matches.min << "/" << stats_num_matches.max << ", threshold (min): " << settings.min_num_valid_pts_ << "</p>\n";
    
    // Init attempts: frames, points, matches, fail reasons in red (table)
    // Fail reason count/histogram
    html << "<table class=\"peebles\">\n";
    html << "    <tr>\n";
    html << "    <th>Frame 0</th>\n";
    html << "    <th>Frame 1</th>\n";
    html << "    <th>Features 0</th>\n";
    html << "    <th>Features 1</th>\n";
    html << "    <th>Unguided matches (min " << settings.min_num_valid_pts_ << ")</th>\n";

    html << "    <th>Guided input matches</th>\n";
    html << "    <th>F-guided inliers (min " << 8 << ")</th>\n"; // 8 is min_set_size for solution_is_valid_ in fundamental_solver::find_via_ransac() - 
    html << "    <th>H-guided inliers (min " << 8 << ")</th>\n"; // 8 is min_set_size for solution_is_valid_ in homography_solver::find_via_ransac()

    html << "    <th>Focal length (pix)</th>\n";
    html << "    <th>Focal stability (min " << 2 << ")</th>\n"; // 2 is set in min_geometric_error_focal_length()
    html << "    <th>Focal length 2-view (pix)</th>\n";
    html << "    <th>Focal length 3-view (pix)</th>\n";

    html << "    <th>Triangulated Points (min " << min_num_valid_triangulated_points.value_or(-1) << ")</th>\n"; // 0 
    html << "    <th>Triangulation Ambiguity (max " << max_triangulation_ambiguity.value_or(-1) << ")</th>\n"; // 1
    html << "    <th>Triangulation Parallax 50 (min " << max_triangulation_parallax.value_or(-1) << "&deg;)</th>\n"; // 2
    html << "    <th>Points with Parallax >0.5&deg; (min " << min_num_triangulated_points.value_or(-1) << ")</th>\n"; // base::triangulate sets a threshold of 0.5 degrees

    //html << "    <th style = \"background-color: yellow\">Data 2 </th>\n";
    html << "   </tr>\n";
    for (auto const& attempt_iter : initialisation_attempts) {
        initialisation_attempt const& attempt(attempt_iter.second);
        html << "   <tr>\n";
        html << "   <td>" << attempt.frame_0 << "</td>\n";
        html << "   <td>" << attempt.frame_1 << "</td>\n";
        html << "   <td>" << attempt.num_features_0 << "</td>\n";
        html << "   <td>" << attempt.num_features_1 << "</td>\n";
        html << "   <td" << style_if_less(attempt.num_unguided_matches, (int)settings.min_num_valid_pts_) << ">" << attempt.num_unguided_matches << "</td>\n";
        //html << "   <td style = \"background-color: yellow\">fOrange</td>\n";

        html << "   <td>" << table_value(attempt.homography_fundamental_candidates) << "</td>\n";
        html << "   <td" << noncritical_style_if_less(attempt.fundamental_inliers, 8) << ">" << table_value(attempt.fundamental_inliers) << "</td>\n";
        html << "   <td" << noncritical_style_if_less(attempt.homography_inliers, 8) << ">" << table_value(attempt.homography_inliers) << "</td>\n";

        html << "   <td>" << table_value(attempt.focal_length_estimate) << "</td>\n";
        html << "   <td" << style_if_less(attempt.focal_length_stability, 2) << ">" << table_value(attempt.focal_length_stability) << "</td>\n";

        html << "   <td>" << table_value(attempt.focal_length_estimate_2_view) << "</td>\n";
        html << "   <td>" << table_value(attempt.focal_length_estimate_3_view) << "</td>\n";

        html << "   <td" << style_if_less(attempt.num_valid_triangulated_points, min_num_valid_triangulated_points) << ">" << table_value(attempt.num_valid_triangulated_points) << "</td>\n"; // 0
        html << "   <td" << style_if_greater(attempt.triangulation_ambiguity, max_triangulation_ambiguity) << ">" << table_value(attempt.triangulation_ambiguity) << "</td>\n"; // 1
        html << "   <td" << style_if_less(attempt.triangulation_parallax, max_triangulation_parallax) << ">" << table_value(attempt.triangulation_parallax) << "</td>\n"; // 2
        html << "   <td" << style_if_less(attempt.num_triangulated_points, min_num_triangulated_points) << ">" << table_value(attempt.num_triangulated_points) << "</td>\n"; // 3

        html << "   </tr>\n";

    }
    html << "    </table>\n";
}

void metrics::save_html_report(std::string_view const& filename, std::string thumbnail_path_relative, std::string video_path_relative,
    std::optional<double> known_focal_length_x_pixels) const {

    int video_frame_count = solved_frame_count + unsolved_frame_count;

    std::ofstream myfile;
    myfile.open(filename.data());

    std::stringstream html;
    html << "<!DOCTYPE html><html>\n";
    html << "<head>\n";
    html << "  <meta charset = 'utf-8' />\n";
    html << "  <title>" << input_video_metadata.name << "</title>\n";
    html << "  <link rel = \" stylesheet \" href = \" default.css \" />\n";
    html << "  <style>\n";
    html << "* { font-family: Arial, Helvetica, sans-serif; }\n";
    html << "mark.red { color:#ff0000; background: none; }\n";
    html << "mark.blue { color:#0000A0; background: none; }\n";

    html << "html{\n";
        html << "font - family: sans - serif;\n";
        html << "}\n";

        html << ".peebles table{\n";
        html << "border - collapse: collapse;\n";
        html << "border: 2px solid rgb(200,200,200);\n";
        html << "letter - spacing: 1px;\n";
        html << "font - size: 0.8rem;\n";
        html << "}\n";

        html << ".peebles td, .peebles th{\n";
        html << "border: 1px solid rgb(190,190,190);\n";
        html << "padding: 10px 20px;\n";
        html << "}\n";

        html << ".peebles th{\n";
        html << "  background - color: rgb(235,235,235);\n";
        html << "}\n";

        html << ".peebles td{\n";
        html << "  text - align: center;\n";
        html << "}\n";

        html << "tr:nth - child(even) .peebles td {\n";
        html << "background - color: rgb(250, 250, 250);\n";
        html << "}\n";

        html << "tr:nth - child(odd) .peebles td {\n";
        html << "background - color: rgb(245, 245, 245);\n";
        html << "}\n";

        html << ".peebles caption{\n";
        html << "padding: 10px;\n";
        html << "}\n";

        html << ".outside_threshold{ background:#DD7575; }\n";
        html << ".outside_threshold_noncritical{ background: repeating-linear-gradient( 45deg, #DD7575, #DD7575 10px, #FFFFFF 10px, #FFFFFF 20px );}\n";

        add_graph_style(html);

    html << "  </style>\n";
    html << "</head>\n";

    html << "<body>\n";

    html << "<h1>" << input_video_metadata.name << "</h1>\n";

    if (!thumbnail_path_relative.empty())// && video_path_relative.empty())
        html << "<img src=\"" << thumbnail_path_relative << "\" alt=\"Preview\" width=\"800\">\n ";

    if (!video_path_relative.empty()) {
        html << "<video width = \"1280\" controls>\n";
        html << "<source src = \"" << video_path_relative << "\" type = \"video/mp4\">\n";
        html << "Your browser does not support HTML video.\n";
        html << "</video>\n";
    }

    html << "<p>Video file: " << input_video_metadata.filename << "</p>\n";
    html << "<p>Video size: " << input_video_metadata.video_width << " x " << input_video_metadata.video_height << " pixels.</p>\n";
    html << "<p>Feature detector adaptive min size scale: " << feature_min_size_scale << ".</p>\n";
    


    html << "<h2>Parameters</h2>\n";
    if (known_focal_length_x_pixels)
        html << "<p> Calculation uses known focal length of " << known_focal_length_x_pixels.value() << " pixels.</p>\n";
    else
        html << "<p> Calculation run with unknown focal length.</p>\n";

    html << "<h2>Camera Intrinsics</h2>\n";

    // Pixel aspect ratio
    html << "<p> Pixel aspect ratio used: " << input_video_metadata.pixel_aspect_ratio_used;
    if (input_video_metadata.ground_truth_pixel_aspect_ratio()) {
        double diff_percent = percent_difference(input_video_metadata.ground_truth_pixel_aspect_ratio().value(),
                                                 input_video_metadata.pixel_aspect_ratio_used);
        if (diff_percent > par_percent_error_trigger)
            html << " (ground truth " << input_video_metadata.ground_truth_pixel_aspect_ratio().value() << " - <mark class=\"red\">  " << diff_percent << "\% difference</mark>)</p>\n";
        else
            html << " (ground truth " << input_video_metadata.ground_truth_pixel_aspect_ratio().value() << " - " << diff_percent << "\% difference)</p>\n";
    }
    else
        html << ".</p>\n";

    // Focal length x pixels
    if (known_focal_length_x_pixels) {
        html << "<p> Fixed focal length of " << known_focal_length_x_pixels.value() << " pixels used.</p>\n";
    }
    else {
        html << "<p> Focal length (pixels) calculated: " << calculated_focal_length_x_pixels;
        if (input_video_metadata.ground_truth_focal_length_x_pixels()) {
            double diff_percent = percent_difference(input_video_metadata.ground_truth_focal_length_x_pixels().value(),
                calculated_focal_length_x_pixels);
            if (diff_percent > focal_percent_error_trigger)
                html << " (ground truth " << input_video_metadata.ground_truth_focal_length_x_pixels().value() << " - <mark class=\"red\">  " << diff_percent << "\% difference</mark>)</p>\n";
            else
                html << " (ground truth " << input_video_metadata.ground_truth_focal_length_x_pixels().value() << " - " << diff_percent << "\% difference)</p>\n";
        }
        else
            html << ".</p>\n";
    }

    // Focal length mm
    if (known_focal_length_x_pixels) {
        if (input_video_metadata.calculated_focal_length_mm(known_focal_length_x_pixels.value()))
            html << "<p> Fixed focal length of " << input_video_metadata.calculated_focal_length_mm(known_focal_length_x_pixels.value()).value() << " mm used.</p>\n";
    }
    else {
        if (input_video_metadata.calculated_focal_length_mm(calculated_focal_length_x_pixels)) {
            html << "<p> Focal length mm calculated: " << input_video_metadata.calculated_focal_length_mm(calculated_focal_length_x_pixels).value();
            if (input_video_metadata.groundTruthFocalLengthMM) {
                double diff_percent = percent_difference(input_video_metadata.groundTruthFocalLengthMM.value(),
                    input_video_metadata.calculated_focal_length_mm(calculated_focal_length_x_pixels).value());
                if (diff_percent > focal_percent_error_trigger)
                    html << " (ground truth " << input_video_metadata.groundTruthFocalLengthMM.value() << " - <mark class=\"red\">  " << diff_percent << "\% difference</mark>)</p>\n";
                else
                    html << " (ground truth " << input_video_metadata.groundTruthFocalLengthMM.value() << " - " << diff_percent << "\% difference)</p>\n";
            }
            else
                html << ".</p>\n";
        }
    }

    html << "<h2> Map</h2>\n";
    // Solved/unsolved cameras
    html << "<p> " << solved_frame_count << " cameras created from the " << video_frame_count << " frames of video";
    if (unsolved_frame_count != 0)
        html << " - <mark class=\"red\"> \<" << unsolved_frame_count << " frames not tracked\></mark></p>\n";
    html << "<p> " << num_points << " 3D points</p>\n";
    if (initialisation_frames.empty())
        html << "<p> No successful initialisations</p>\n";
    for (auto const& initialisation_attempt_frames : initialisation_frames)
        html << "<p> Initialisation Frames: " << print_initialisation(initialisation_attempt_frames, intermediate_focal_estimates) << "</p>\n";
    if (initialisation_debug_object.average_init_frame_feature_count())
        html << "<p> Initialisation average feature count: " << initialisation_debug_object.average_init_frame_feature_count().value() << "</p>\n";
    if (initialisation_debug_object.average_init_frame_unguided_match_count())
        html << "<p> Initialisation average match count: " << initialisation_debug_object.average_init_frame_unguided_match_count().value() << "</p>\n";

    // Map growth
    html << "<h2> Map Growth</h2>\n";
    for (auto const& candidate_map : candidate_map_list)
        if (candidate_map.abandoned)
            html << "<p>Rejected map " << candidate_map.frame_range.first << "-" << candidate_map.frame_range.second << " (" << candidate_map.key_count << " keyframes, " << candidate_map.fail_count << " fails) " << (candidate_map.selected ? " (reinstated for second pass)" : "") << "</p>\n";
        else
            html << "<p>Final map " << candidate_map.frame_range.first << "-" << candidate_map.frame_range.second << " (" << candidate_map.key_count << " keyframes, " << candidate_map.fail_count << " fails) " << (candidate_map.selected ? " (used for second pass)" : "") << "</p>\n";
    html << "<p>Second pass added " << pass_2_end_keyframes - pass_1_end_keyframes << " keyframes from " << pass_2_frames << " frames</p>\n";
    html << "<p>Final pass created " << solved_frame_count << " camera positions from " << video_frame_count << " frames</p>\n";

    {
        std::optional<double> focal_gt(input_video_metadata.ground_truth_focal_length_x_pixels());
        axis_scaling y_axis_scaling = focal_gt ? axis_scaling(1.2 * focal_gt.value()) : range_behaviour::no_max;
        write_graph_as_svg(html, Graph("Second Frame", "Focal length", std::set<SplitCurve>({ {"2-view estimate", initialisation_focal_length_estimate_2_view.graph()},
                                                                                          {"3-view estimate", initialisation_focal_length_estimate_3_view.graph()} }), range_behaviour::split_by_stage, y_axis_scaling, focal_gt));
    }

    std::list<curve_section> graph_num_matches = select_second_frame_data_for_curves(initialisation_debug_object.p_num_matches.by_frame, 0);
    int stage(0);
    //curve_section graph_feature_count = detected_feature_count.graph();
    //curve_section graph_feature_count;
    //for (auto const& i : initialisation_debug_object.feature_count_by_frame)
    //    graph_feature_count[i.first] = i.second;
    //graph_feature_count.stage = 0;
    //graph_num_matches.stage = 0;
    add_matching_details(html, detected_feature_count.graph(), graph_num_matches);


    // Timing
    html << "<h2> Timing</h2>\n";
    std::optional<double> fps(video_frame_count == 0 ? std::nullopt : std::optional<double>((double(video_frame_count) / track_timings.total_time_sec())));
    html << "<p> Total tracking time:" << track_timings.total_time_sec() << " sec";
    if (fps)
        html << " (" << fps.value() << "fps)";
    html << "</p>\n ";
    html << "<p> Forward mapping: " << track_timings.forward_mapping << ".</p>\n";
    html << "<p> Backward mapping: " << track_timings.backward_mapping << ".</p>\n";
    html << "<p> Loop Closing: " << track_timings.loop_closing << ".</p>\n";
    html << "<p> Optimisation: " << track_timings.optimisation << ".</p>\n";
    html << "<p> Final Tracking: " << track_timings.tracking << ".</p>\n";

    if (debugging.debug_initialisation) {
        html << "<h2>Initialisation debug</h2>\n";
        initialisation_debug_object.add_to_html(html, input_video_metadata.ground_truth_focal_length_x_pixels());
    }

    // Number of features and matches by frame
    html << "<h2> Feature match counts</h2>\n";
    write_graph_as_svg(html, Graph("Second init frame", "Num feature matches", std::set<SplitCurve>({ {"Feature count", detected_feature_count.graph()}, {"Matches to frame", graph_num_matches} }), range_behaviour::no_max, range_behaviour::no_max, settings.min_num_valid_pts_));
    write_graph_as_svg(html, Graph("Frame", "min feature size", std::set<SplitCurve>({ {"Min feature size", min_feature_size.graph()} })));
    html << "<hr>" << std::endl;

    // Tracking stats
    html << "<p>Tracking: ";
    write_graph_as_svg(html, Graph("Frame", "Motion Match Count", std::set<SplitCurve>({ {"tracking_motion_inputs_A", tracking_motion_inputs_A.graph()},
                {"tracking_motion_inputs_B", tracking_motion_inputs_B.graph()},
                {"tracking_motion_matches_1", tracking_motion_matches_1.graph()} ,
                {"tracking_motion_matches_2", tracking_motion_matches_2.graph()} ,
                {"tracking_motion_matches_optimised", tracking_motion_matches_optimised.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max, tracking_motion_matches_1.threshold));
    write_graph_as_svg(html, Graph("Frame", "BOW Match Count", std::set<SplitCurve>({ {"tracking_bow_inputs_A", tracking_bow_inputs_A.graph()},
                {"tracking_bow_inputs_B", tracking_bow_inputs_B.graph()},
                {"tracking_bow_matches", tracking_bow_matches.graph()} ,
                {"tracking_bow_matches_optimised", tracking_bow_matches_optimised.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max, tracking_bow_matches.threshold));
    write_graph_as_svg(html, Graph("Frame", "robust Match Count", std::set<SplitCurve>({ {"tracking_robust_inputs_A", tracking_robust_inputs_A.graph()},
                {"tracking_robust_inputs_B", tracking_robust_inputs_B.graph()},
                {"tracking_robust_matches", tracking_robust_matches.graph()} ,
                {"tracking_robust_matches_optimised", tracking_robust_matches_optimised.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max, tracking_robust_matches.threshold));

    // Structure type (planar or non-planar) costs during two-view matching 
    std::map<double, double> graph_cost_H = select_second_frame_data(initialisation_debug_object.p_cost_H.by_frame);
    std::map<double, double> graph_cost_F = select_second_frame_data(initialisation_debug_object.p_cost_F.by_frame);
    html << "<h2>Structure type</h2>" << std::endl;
    double cost_percentile90 = 1.1 * std::max(percentile_y_value(graph_cost_H, 0.9), percentile_y_value(graph_cost_F, 0.9));

    std::set<SplitCurve> cost_curves = { { "Planar error", { curve_section(graph_cost_H, 0) } },
                                         { "Non-planar error", { curve_section(graph_cost_F, 0) } } };


    //for (auto const& curve : cost_curves) {
    //    html << " curve  " << curve.first << "\n";
    //    for (auto const& g : curve.second) {
    //        html << " struct  {\n";
    //        html << " stage =  " << g.stage << "\n";
    //        for (auto const& i : g)
    //            html << "   {" << i.first << ", " << i.second << "},\n";
    //        html << "}\n";
    //    }
    //}

    //write_graph_as_svg(html, Graph("Second init frame", "Cost", std::set<Curve>({ {"Planar error", graph_cost_H}, {"Non-planar error", graph_cost_F} }), range_behaviour::no_max, cost_percentile90, std::nullopt));
    write_graph_as_svg(html, Graph("Second init frame", "Cost", cost_curves, range_behaviour::no_max, cost_percentile90, std::nullopt));

    html << "<p>Average feature match deviation from a planar or non-planar geometric model (pixels - with max of a few pixels).</p>" << std::endl;
    html << "<hr>" << std::endl;

    // Intermediate focal length estimates
    html << "<h2> Intermediate focal length estimates</h2>\n";
    if (!known_focal_length_x_pixels) {

        std::array<std::map<int,curve_section>, 4> graph_intermediate_focal_length_estimate;
        for (auto const& estimate : intermediate_focal_estimates) {
            int stage = estimate.stage_with_frame.stage; // First pass, second, etc
            focal_estimation_type type = estimate.type; // Calculation type unoptimised init, local optimisation etc - same curve
            graph_intermediate_focal_length_estimate[static_cast<unsigned int>(type)][stage].stage = stage;
            graph_intermediate_focal_length_estimate[static_cast<unsigned int>(type)][stage][estimate.stage_with_frame.frame] = estimate.estimate;
        }

        //{
        //    // Convert to a map<double,double> for presentation as a graph
        //    std::set<SplitCurve> curves;
        //    for (unsigned int i = 0; i < 4; ++i)
        //        curves.insert({ focal_estimation_type_to_string.at(i), split_graph_by_frame_set(graph_intermediate_focal_length_estimate[i], initialisation_frames) });


        //    std::optional<double> focal_gt(input_video_metadata.ground_truth_focal_length_x_pixels());
        //    axis_scaling y_axis_scaling = focal_gt ? axis_scaling(2.0 * focal_gt.value()) : range_behaviour::no_max;
        //    write_graph_as_svg(html, Graph("Frame", "Focal length estimate", curves, range_behaviour::no_max, y_axis_scaling, focal_gt));
        //}

        {
            // Convert to a map<double,double> for presentation as a graph
            std::set<SplitCurve> curves;
            for (unsigned int i = 0; i < 4; ++i)
                curves.insert({ focal_estimation_type_to_string.at(i), split_graph_by_frames(focal_estimation_type_to_string.at(i), graph_intermediate_focal_length_estimate[i], mapping_reset_frames) });

            std::optional<double> focal_gt(input_video_metadata.ground_truth_focal_length_x_pixels());
            axis_scaling y_axis_scaling = focal_gt ? axis_scaling(2.0 * focal_gt.value()) : range_behaviour::no_max;
            write_graph_as_svg(html, Graph("Frame", "Focal length estimate", curves, range_behaviour::split_by_stage, y_axis_scaling, focal_gt));
        
            double percentile90 = percentile_y_value(graph_intermediate_focal_length_estimate, 0.9);
            write_graph_as_svg(html, Graph("Frame", "Focal length estimate", curves, range_behaviour::split_by_stage, percentile90, focal_gt));
        
            //for (auto const& curve : curves) {
            //    html << " curve  " << curve.first << "\n";
            //    for (auto const& g : curve.second) {
            //        html << " struct  {\n";
            //        html << " stage =  " << g.stage << "\n";
            //        for (auto const& i : g)
            //            html << "   {" << i.first << ", " << i.second << "},\n";
            //        html << "}\n";
            //    }
            //}
        
        
        }

        //for (unsigned int i = 0; i < 4; ++i) {
        //    std::set<std::map<double, double>> s = split_graph_by_frames(focal_estimation_type_to_string.at(i), graph_intermediate_focal_length_estimate[i], mapping_reset_frames);
        //    for (auto const& m : s) {
        //        html << "<p>Split " << focal_estimation_type_to_string.at(i) << " ";
        //        for (auto const& i : m)
        //            html << i.first << ", ";
        //    }
        //}

        //for (auto const& estimate : intermediate_focal_estimates)
        //    html << "<p>Intermediate focal estimate [" << focal_estimation_type_to_string.at(static_cast<unsigned int>(estimate.stage)) << "] at " << estimate.frame << " is " << estimate.estimate;
    }

    // Mapping reset frames
    html << "<p>Reset frames: ";
    for (auto const& reset_frame : mapping_reset_frames)
        html << reset_frame << ", ";
    write_graph_as_svg(html,  Graph("Frame", "Map keyframe count", std::set<SplitCurve>({ {"Num keyframes", map_size.graph()} }), range_behaviour::split_by_stage));
    
    //auto map_size_graph = map_size.graph();
    //for (auto const& g : map_size_graph) {
    //    html << " struct  {\n";
    //    for (auto const& i : g)
    //        html << "   {" << i.first << ", " << i.second << "},\n";
    //    html << "}\n";
    //}

    write_graph_as_svg(html, Graph("Frame", "Fail count", std::set<SplitCurve>({ {"Fail count", tracking_fail_count.graph()} }), range_behaviour::split_by_stage));
    //axis_scaling(input_video_metadata.end_frame)

    write_graph_as_svg(html, Graph("Frame #2", "Count", std::set<SplitCurve>({ {"Frame 1 points", area_match_frame_1_points.graph()},
                                                                               {"Fail prematched", area_match_fail_prematched.graph()},
                                                                               {"Num attempted", area_match_num_attempted.graph()},
                                                                                {"Feature count", { detected_feature_count.graph() } },
                                                                                {"Matches to frame", { graph_num_matches } },
                                                                               {"Fail scale", area_match_fail_scale.graph()},
                                                                               {"Fail cell", area_match_fail_cell.graph()},
                                                                               {"Fail hamming", area_match_fail_hamming.graph()},
                                                                               {"Fail ratio", area_match_fail_ratio.graph()},
                                                                               {"Valid point matches", area_match_num_matches.graph()} }),
        range_behaviour::split_by_stage, range_behaviour::no_max));
    write_graph_as_svg(html, Graph("Frame #2", "Ave candidates", std::set<SplitCurve>({ {"Ave candidates", area_match_ave_candidates.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max));


    write_graph_as_svg(html, Graph("Frame #2", "Valid Points (match rejected if below threshold)", std::set<SplitCurve>({ {"Valid Points", num_valid_triangulated_points.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max, min_num_valid_triangulated_points));
    write_graph_as_svg(html, Graph("Frame #2", "Ambiguity (match rejected if above threshold)", std::set<SplitCurve>({ {"Ambiguity", triangulation_ambiguity.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max, max_triangulation_ambiguity));
    write_graph_as_svg(html, Graph("Frame #2", "Parallax (match rejected if below threshold)", std::set<SplitCurve>({ {"Parallax", triangulation_parallax.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_min_0, max_triangulation_parallax));
    write_graph_as_svg(html, Graph("Frame #2", "Triangulated Points (match rejected if below threshold)", std::set<SplitCurve>({ {"Points", num_triangulated_points.graph()} }), range_behaviour::split_by_stage, range_behaviour::no_max, min_num_triangulated_points));
    write_graph_as_svg(html, Graph("Frame #2", "Frames of separation", std::set<SplitCurve>({ {"Separation", triangulation_parallax.frame_separation_graph()} }), range_behaviour::split_by_stage, range_behaviour::no_min_0));

    // Print the config settings
    std::stringstream ss_settings;
    ss_settings << settings;
    std::vector<std::string> split_settings = splitString(ss_settings.str());
    html << "<h2> Settings</h2>\n";
    for (auto const& settings_line : split_settings)
        if (settings_line.rfind("\t", 0) == 0)
            html << "<p  style=\"margin-left: 40px\">" << settings_line << "</p>";
        else
            html << "<h3  style=\"margin-left: 20px\">" << settings_line << "</h3>";

    html << "</body></html>";

    myfile << html.str();
    myfile.close();
}

void metrics::save_json_report(std::string_view const& filename) const
{
    std::ofstream jsonFileStream(filename.data());
    jsonFileStream << std::setw(4) << to_json() << std::endl;
}

void metrics::save_html_overview(std::string_view const& filename,
                                 std::list<track_test_info> const& track_test_info_list,
                                 bool initialisation_debug_test,
                                 bool known_focal_length_test) {
    std::ofstream myfile;
    myfile.open(filename.data());

    std::stringstream html;


	html << "<!DOCTYPE html> \n";
    html << "<html> \n";
    html << "<head> \n";
    html << "    <meta charset='utf-8' /> \n";
    html << "    <title>Camera Tracking Evaluation Run Overview</title> \n";
    html << "    <link rel=\" stylesheet \" href=\" default.css \" /> \n";
    html << "	<style>\n";
    html << "* { font-family: Arial, Helvetica, sans-serif; }\n";
    html << "#wrap > div { overflow:hidden; }\n";

    html << "#testruns warn { \n";
    html << "font-size:12px; \n";
//    html << "    color:#FF6A00; \n";
    html << "    font-style:normal; \n";
    html << "	display:inline-block; \n";
    html << "}	\n";
    html << "#testruns error { \n";
    html << "font-size:12px; \n";
    html << "    color:#FF0000; \n";
    html << "    font-style:normal; \n";
    html << "	display:inline-block; \n";
    html << "}	\n";
    html << "#testruns pass { \n";
    html << "    font-style:normal; \n";
    html << "    background:#00cf00; \n";
    html << "    margin-right:10px; \n";
    html << "    display:inline-block; \n";
    html << "    width:32px; \n";
    html << "    padding:0 10px; \n";
    html << "    border-radius:15px; \n";
    html << "    -moz-border-radius:15px; \n";
    html << "    -webkit-border-radius:15px; \n";
    html << "}\n";
    html << "#testruns fail { \n";
    html << "    font-style:normal; \n";
    html << "    background:#cfcfcf;\n";
    html << "    margin-right:10px; \n";
    html << "    display:inline-block; \n";
    html << "    width:32px; \n";
    html << "    padding:0 10px; \n";
    html << "    border-radius:15px; \n";
    html << "    -moz-border-radius:15px; \n";
    html << "    -webkit-border-radius:15px; \n";
    html << "}\n";
    html << "#testruns text { \n";
    html << "    font-style:normal; \n";
    html << "    display:inline-block; \n";
    html << "} 		\n";

    html << ".table {\n";
    html << "  display: table;\n";
    html << "}\n";

    html << ".row {\n";
    html << "  display: table-row;\n";
    html << "}\n";

    html << ".cell {\n";
    html << "  display: table-cell;\n";
    html << "  padding: 10px;\n";
    html << "}\n";

    html << ".row:hover {\n";
    html << "  background-color: #cccccc;\n";
    html << "}\n";

    //html << "h1 { margin:20; padding:20; font-size:40px; } \n";
    //html << "h1   {color: blue;}\n";
    html << "    h1    {color: #0088CC;}\n";
    html << "    p    {color: #0088CC;}\n";
    html << "		</style>\n";

    html << "<link href = \"https://stackpath.bootstrapcdn.com/twitter-bootstrap/2.3.2/css/bootstrap-combined.min.css\" rel=\"stylesheet\" type = \"text/css\" />\n ";



    html << "	</head> \n";

    html << "	<body> \n";

    html << "		<h1>Camera tracking evaluation run</h1> \n";

    html << "		<p style=\"margin-left: 10px\">  Test run id: b6dafc8e-6c2b-478b-8dfe-55df227e8739 </p> \n";
    if (initialisation_debug_test)
        html << "		<p style=\"margin-left: 10px\">Initialisation debugging test</p> \n";
    if (known_focal_length_test)
        html << "		<p style=\"margin-left: 10px\">Known focal length test</p> \n";
    else
        html << "		<p style=\"margin-left: 10px\">Auto focal length test</p> \n";

    int pass_count(0), warn_count(0), fail_count(0);
    for (auto const& test_info : track_test_info_list) {
        if (test_info.m->max_problem_level() == std::nullopt)
            ++pass_count;
        if (test_info.m->max_problem_level() == tracking_problem_warning)
            ++warn_count;
        if (test_info.m->max_problem_level() == tracking_problem_fatal)
            ++fail_count;
    }

    html << "		<p style=\"margin-left: 10px\">  Pass " << pass_count << ", warning " << warn_count << ", fail " << fail_count << "</p> \n";

    html << "<hr>\n";

    html << "<div id=\"testruns\" role=\"grid\" class=\"table\">\n";

    for (auto const& test_info : track_test_info_list) {
        metrics const* m(test_info.m);
        int video_frame_count = m->solved_frame_count + m->unsolved_frame_count;
        bool fail(m->solved_frame_count == 0);
        std::optional<double> fps(video_frame_count == 0 ? std::nullopt : std::optional<double>((double(video_frame_count) / m->track_timings.total_time_sec())));
        std::string style = fail ? "style=\"background-color: #ffcccc; .hover:background-color: #dc8c8c;\"" : "";

        html << "  <a role=\"row\" class=\"row\" href=\"" << test_info.html_filename << "\"" << style << ">\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      " << (fail ? "<fail>Fail</fail>" : "<pass>Pass</pass>") << "<img src=\"" << test_info.thumbnail_filename << "\" width=\"60\"><text>&nbsp;" << m->input_video_metadata.name;
        html <<  " " << test_info.m->input_video_metadata.video_width << " x " << test_info.m->input_video_metadata.video_height << "</text>\n";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      \n";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      <text>" << video_frame_count << " frames, " << m->track_timings.total_time_sec() << " sec";
        if (fps)
            html << " (" << fps.value() << "fps)";
        html << "</text>\n ";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      <text>" << m->num_points << " points</text>\n";
        html << "    </div>\n";
        html << "    <div role=\"gridcell\" class=\"cell\">\n";
        html << "      ";
        auto problems = m->problems();
        for (auto const& problem : problems)
            if (problem.second==tracking_problem_warning)
               html << "<warn>&lt;" << problem.first << "&gt;</warn>";
            else
               html << "<error>&lt;" << problem.first << "&gt;</error>";
        html << "\n";
        html << "    </div>\n";
        html << "  </a> \n";
    }

    html << "</div>\n";
    html << "	</body> \n";
    html << "	</html>\n";


    myfile << html.str();
    myfile.close();
}

metrics_copy::metrics_copy(metrics const& m)
: m_copy(m)
{
}

metrics const& metrics_copy::operator()() {
    return m_copy;
}

static void fill_dummy_metrics(int i, metrics *m)
{
    using namespace std;
    using opt = vector<optional<double>>;

    if (i >= 3)
        i = 3;

    int num(i + 1);

    // metrics
    m->calculated_focal_length_x_pixels = vector({898.7, 898.7, 1307.2})[i];
    m->solved_frame_count = vector({0, 100, 201})[i];
    m->unsolved_frame_count = vector({76, 5, 0})[i];
    m->num_points = num * 17735;
    m->initialisation_frames = { vector<std::set<int>>({{13, 14}, {2, 23}, {0, 10}})[i] };

    // input video metadata
    m->input_video_metadata.name = vector({"Hillshot7", "Cup Final", "Fishtank"})[i];// A name used in reporting
    m->input_video_metadata.filename = vector({"Hillshot7.mpg", "Cup Final.mpg", "Fishtank.mpg"})[i]; // Name of the filename stored locally (in testdata/Cam3D)
    m->input_video_metadata.gDriveID = vector({"Hillshot7", "Cup Final", "Fishtank"})[i];              // google drive id of the video
    m->input_video_metadata.video_width = 1920;
    m->input_video_metadata.video_height = 1080;
    m->input_video_metadata.start_frame = vector({0, 100, 0})[i];
    m->input_video_metadata.end_frame = vector({77, 201, 200})[i];
    m->input_video_metadata.pixel_aspect_ratio_used = vector({1.5064, 1.0, 1.5064})[i];
    m->input_video_metadata.groundTruthFocalLengthXPixels = opt({{}, {}, {}})[i];
    m->input_video_metadata.groundTruthFilmBackWidthMM = 23.5;
    m->input_video_metadata.groundTruthFilmBackHeightMM = 15.6;
    m->input_video_metadata.groundTruthFocalLengthMM = opt({11, 11, 16})[i];

    // timings
    m->track_timings.forward_mapping = vector({112, 123, 164})[i];
    m->track_timings.backward_mapping = vector({131, 141, 165})[i];
    m->track_timings.loop_closing = vector({0.4, 1.2, 0.0})[i];
    m->track_timings.optimisation = vector({12, 11, 16})[i];
    m->track_timings.tracking = vector({115, 126, 167})[i];

}

void metrics_html_test(std::string const& directory, std::array<std::string, 3> image_filenames)
{
    // create some dummy metrics
    std::array<std::unique_ptr<metrics_copy>, 3> metrics_array;
    std::list<metrics::track_test_info> track_test_info_list;

    metrics* the_metrics = metrics::get_instance();

    // Images in directory
    // Individual html in directory
    // Overview in directory
    std::array<std::string, 3> thumbnail_filenames_relative_html = image_filenames;
    std::array<std::string, 3> thumbnail_filenames_relative_overview = image_filenames;

    for (int i = 0; i < 3; ++i) {
        std::string html_filename_absolute = directory + "/report" + std::to_string(i + 1) + ".html";
        std::string html_filename_relative_overview = "report" + std::to_string(i + 1) + ".html";

        fill_dummy_metrics(i, the_metrics);
        metrics_array[i] = std::make_unique<metrics_copy>(*the_metrics);
        
        track_test_info_list.push_back({&metrics_array[i].get()->operator()(), thumbnail_filenames_relative_overview[i], html_filename_relative_overview});
        the_metrics->save_html_report(html_filename_absolute, thumbnail_filenames_relative_html[i], std::string(), std::nullopt);

        // write prettified JSON to another file
        std::string json_filename = directory + "/report" + std::to_string(i + 1) + ".json";
        std::ofstream out(json_filename);
        out << std::setw(4) << the_metrics->to_json() << std::endl;

        std::ifstream in(json_filename);
        nlohmann::json j;
        in >> j;
    }
    metrics::save_html_overview(directory + "/overview.html", track_test_info_list, false, false);
}

//lookup timestamp->frame & stage
//submit()
//single frame: store map<stage, map<frame, value>>
//frame pair: 
//focal length estimate, map size

} // namespace stella_vslam_bfx
