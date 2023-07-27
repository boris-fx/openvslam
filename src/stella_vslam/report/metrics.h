/** \file
 * \brief Camera tracking metrics recorded during and ater tracking
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <string>
#include <optional>
#include <list>
#include <array>
#include <map>
#include <set>

#include <nlohmann/json_fwd.hpp>

#include "stella_vslam/exports.h"
#include "stella_vslam/config_settings.h"

#include "initialisation_debugging.h"

struct curve_section;

namespace stella_vslam_bfx {

struct STELLA_VSLAM_API video_metadata {

    std::string name;                                    // A name used in reporting
    std::string filename;                                // Name of the filename stored locally (in testdata/Cam3D)
    std::string gDriveID;                                // google drive id of the video
    
    // Video properties
    int    video_width;
    int    video_height;
    int    start_frame;
    int    end_frame;
    double pixel_aspect_ratio_used; // calculated from IS_FilmTypeFactory::bestMatchForSize

    std::optional<double> groundTruthFocalLengthXPixels; // Not used in tracking, but in evaluation
    std::optional<double> groundTruthFilmBackWidthMM;    // Not used in tracking, but in evaluation
    std::optional<double> groundTruthFilmBackHeightMM;   // Not used in tracking, but in evaluation
    std::optional<double> groundTruthFocalLengthMM;      // Not used in tracking, but in evaluation

    nlohmann::json to_json() const;
    bool from_json(const nlohmann::json& json);

    //
    std::optional<double> ground_truth_pixel_aspect_ratio() const;
    std::optional<double> ground_truth_focal_length_x_pixels() const;
    std::optional<double> calculated_focal_length_mm(double calculated_focal_length_x_pixels) const;


};

// A set of bools which can be used to trigger special testing modes which won't necessarily try to solve the camera
struct STELLA_VSLAM_API debugging_setup {
    bool debug_initialisation;

    debugging_setup();

    nlohmann::json to_json() const;
    bool from_json(const nlohmann::json& json);
};

struct STELLA_VSLAM_API timings {

    double forward_mapping;
    double backward_mapping;
    double loop_closing;
    double optimisation; // aka. bundle adjust
    double tracking; // aka. resection

    double total_time_sec() const;

    nlohmann::json to_json() const;
    bool from_json(const nlohmann::json& json);
};

enum tracking_problem_level {
    tracking_problem_warning,
    tracking_problem_fatal
};

enum class focal_estimation_type {
    initialisation_before_ba=0,
    initialisation_after_ba=1,
    local_optimisation=2,
    global_optimisation=3
};
const std::array<std::string, 4> focal_estimation_type_to_string = { {"Unoptimised initialisation", "Optimised initialisation", "Local optimisation", "Global optimisation"} };

/**
 * \brief Metrics for the tracking of one video section
 */
class STELLA_VSLAM_API metrics {
public:

    // todo: add a multithreaded access mutex

    debugging_setup debugging;

    video_metadata input_video_metadata;

    timings track_timings;

    // Basic information about the final solve
    double calculated_focal_length_x_pixels;
    int solved_frame_count;
    int unsolved_frame_count;
    int num_points;

    std::set<std::set<int>> initialisation_frames; // Multiple sets if multiple initialisations happened

    std::map<double, stage_and_frame> *timestamp_to_stage_and_frame;

    stella_vslam_bfx::config_settings settings;

    void submit_intermediate_focal_estimate(focal_estimation_type stage, double estimate);

    void submit_map_size_and_tracking_fails(double timestamp, unsigned int map_keyframe_count, unsigned int tracking_fails);

    bool capture_area_matching = false;
    void submit_area_matching(int frame_1_points, int fail_prematched, int fail_scale, int fail_cell, int fail_hamming, int fail_ratio,
                              int count_indices_exist, int count_num_indices, int num_matches);

    // First element in each pair is the value, second is the threshold
    void submit_triangulation_debugging(std::optional<std::pair<double, double>> num_triangulated_points, // threshold is a minimum
                                        std::optional<std::pair<double, double>> num_valid_triangulated_points, // threshold is a minimum
                                        std::optional<std::pair<double, double>> triangulation_parallax, // threshold is a maximum
                                        std::optional<std::pair<double, double>> triangulation_ambiguity); // threshold is a maximum
    void submit_mapping_reset(double timestamp);

    void submit_initialiser_constrained_matching_stats(std::vector<bool> const& homography_inliers, std::vector<bool> const& fundamental_inliers);

    void submit_focal_length_estimate(double focal_length, double stability);

    void submit_2_3_view_focal_length(std::optional<double> focal_length_2_view,
                                      std::optional<double> focal_length_3_view);

    // Tracking stats
    //void submit_tracking_by_motion_matches_1(int num_matches, int min_num_matches_threshold); // Initial motion matches
    //void submit_tracking_by_motion_matches_2(int num_matches, int min_num_matches_threshold); // Motion matches further out if first matching fails
    //void submit_tracking_by_motion_matches_optimised(int num_matches, int min_num_matches_threshold); // Motion matches after pose optimisation
    //void submit_tracking_by_bow_matches(int num_matches, int min_num_matches_threshold); // Initial bow matches
    //void submit_tracking_by_bow_matches_optimised(int num_matches, int min_num_matches_threshold); // BOW matches after pose optimisation
    //void submit_tracking_by_robust_matches(int num_matches, int min_num_matches_threshold); // Initial robust matches
    //void submit_tracking_by_robust_matches_optimised(int num_matches, int min_num_matches_threshold); // Robust matches after pose optimisation

    double current_frame_timestamp;

    // Convert the timestamped metrics to frame numbers
    void create_frame_metrics();

    std::set<std::pair<std::string, tracking_problem_level>> problems() const;
    std::optional<tracking_problem_level> max_problem_level() const;

    std::optional<stage_and_frame> timestamp_to_frame(double timestamp);

    float feature_min_size_scale;

    struct candidate_map_stats { std::pair<int, int > frame_range; int key_count; int fail_count; bool abandoned; bool selected; };
    std::list<candidate_map_stats> candidate_map_list;

    int pass_1_end_keyframes;
    int pass_2_frames;
    int pass_2_end_keyframes;

public:
    std::set<std::set<double>> initialisation_frame_timestamps; /// For tracker internal use

    static const double par_percent_error_trigger;
    static const double focal_percent_error_trigger;
    
    metrics& operator=(const metrics&) = delete;

    static metrics* get_instance();
    static metrics* instance();
    static void clear(); // clear for a new camera track

    static initialisation_debugging& initialisation_debug();

    nlohmann::json to_json() const;
    bool from_json(const nlohmann::json& json);

    void save_html_report(std::string_view const& filename, std::string thumbnail_path_relative, std::string video_path_relative, std::optional<double> known_focal_length_x_pixels) const;
    void save_json_report(std::string_view const& filename) const;
    
    struct track_test_info {
        metrics const* m;
        std::string thumbnail_filename; // thumbnail image
        std::string html_filename; // html for the track
    };
    static void save_html_overview(std::string_view const& filename,
                                   std::list<track_test_info> const& track_test_info_list,
                                   bool initialisation_debug_test, bool known_focal_length_test);


    struct focal_estimate
    {
        double estimate;
        focal_estimation_type type;
        stage_and_frame stage_with_frame; // tracking stage and frame
    };

    struct stage_and_frame_param_base {
    };
    template<typename T>
    struct stage_and_frame_param : public stage_and_frame_param_base {
        std::array<std::map<int, T>, max_stage> by_stage_and_frame;
        std::list<curve_section> graph() const;
    };

    template<typename T>
    struct stage_and_frame_param_with_threshold : public stage_and_frame_param<T> {
        T threshold = T(0);
    };

    template<typename T>
    struct stage_and_frame_pair_param {
        std::array<std::map<std::pair<int, int>, T>, max_stage> by_stage_and_frame;
        std::list<curve_section> graph() const; // Graph of frame 2 against value
        std::list<curve_section> frame_separation_graph() const;  // Graph of frame 2 against 'frame 2' - 'frame 1'
    };

    std::list<std::pair<std::string, metrics::stage_and_frame_param_base*>> stage_and_frame_param_base_list();

public:

    template<typename Tp, typename T> static void submit_frame_param(stage_and_frame_param_with_threshold<Tp>& param, T value, T threshold);
    template<typename Tp, typename T> static void submit_frame_param(stage_and_frame_param<Tp>& param, T value);

    // Feature detection and monitoring
    stage_and_frame_param<unsigned int> detected_feature_count;
    stage_and_frame_param<unsigned int> min_feature_size;

    // Motion-based track, then bow match based track, then robust_match (tracking_module::track_current_frame)
    stage_and_frame_param<unsigned int> tracking_motion_inputs_A; // Number of points in the map being matched to the new frame
    stage_and_frame_param<unsigned int> tracking_motion_inputs_B; // Number of features in the new frame being matched
    stage_and_frame_param_with_threshold<unsigned int> tracking_motion_matches_1; // Matches
    stage_and_frame_param_with_threshold<unsigned int> tracking_motion_matches_2; // Matches further out if first matching fails
    stage_and_frame_param_with_threshold<unsigned int> tracking_motion_matches_optimised; // Matches after pose optimisation

    stage_and_frame_param<unsigned int> tracking_bow_inputs_A; // Number of points in the map being matched to the new frame
    stage_and_frame_param<unsigned int> tracking_bow_inputs_B; // Number of features in the new frame being matched
    stage_and_frame_param_with_threshold<unsigned int> tracking_bow_matches; // Matches
    stage_and_frame_param_with_threshold<unsigned int> tracking_bow_matches_optimised; // Matches after pose optimisation

    stage_and_frame_param<unsigned int> tracking_robust_inputs_A; // Number of points in the map being matched to the new frame
    stage_and_frame_param<unsigned int> tracking_robust_inputs_B; // Number of features in the new frame being matched
    stage_and_frame_param_with_threshold<unsigned int> tracking_robust_matches; // Matches
    stage_and_frame_param_with_threshold<unsigned int> tracking_robust_matches_optimised; // Matches after pose optimisation

protected:

    void add_matching_details(std::stringstream& html, std::list<curve_section> const& graph_feature_count, std::list<curve_section> const& graph_num_matches) const;

    

protected:

    std::list<focal_estimate> intermediate_focal_estimates;

    stage_and_frame_param<unsigned int> map_size;
    stage_and_frame_param<unsigned int> tracking_fail_count;

    stage_and_frame_pair_param<double> area_match_frame_1_points;
    stage_and_frame_pair_param<double> area_match_fail_prematched;
    stage_and_frame_pair_param<double> area_match_fail_scale;
    stage_and_frame_pair_param<double> area_match_fail_cell;
    stage_and_frame_pair_param<double> area_match_fail_hamming;
    stage_and_frame_pair_param<double> area_match_fail_ratio;
    stage_and_frame_pair_param<double> area_match_num_matches;
    stage_and_frame_pair_param<double> area_match_num_attempted;
    stage_and_frame_pair_param<double> area_match_ave_candidates;

    stage_and_frame_pair_param<double>  num_triangulated_points; // 3
    stage_and_frame_pair_param<double>  num_valid_triangulated_points; // 0
    stage_and_frame_pair_param<double>  triangulation_parallax; // 2
    stage_and_frame_pair_param<double>  triangulation_ambiguity; // 1

    std::optional<double> min_num_triangulated_points; // 3
    std::optional<double> min_num_valid_triangulated_points; // 0
    std::optional<double> max_triangulation_parallax; // 2
    std::optional<double> max_triangulation_ambiguity; // 1

    stage_and_frame_pair_param<double>  initialisation_homography_fundamental_candidates;
    stage_and_frame_pair_param<double>  initialisation_homography_inliers;
    stage_and_frame_pair_param<double>  initialisation_fundamental_inliers;

    stage_and_frame_pair_param<double>  initialisation_focal_length_estimate;
    stage_and_frame_pair_param<double>  initialisation_focal_length_stability;

    stage_and_frame_pair_param<double>  initialisation_focal_length_estimate_2_view;
    stage_and_frame_pair_param<double>  initialisation_focal_length_estimate_3_view;



    std::list<double> mapping_reset_timestamps;
    std::list<int> mapping_reset_frames;

    friend class metrics_copy;

    initialisation_debugging initialisation_debug_object;

    inline static metrics* m_instance{nullptr};
    metrics() = default;
    ~metrics() = default;
    metrics(const metrics&) = default;
};

template<typename Tp, typename T>
void metrics::submit_frame_param(stage_and_frame_param_with_threshold<Tp>& param, T value, T threshold) {
    std::optional<stage_and_frame> stage_with_frame = instance()->timestamp_to_frame(instance()->current_frame_timestamp);
    if (!stage_with_frame)
        return;
    param.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = (Tp)value;
    param.threshold = (Tp)threshold;
}

template<typename Tp, typename T>
void metrics::submit_frame_param(stage_and_frame_param<Tp>& param, T value) {
    std::optional<stage_and_frame> stage_with_frame = instance()->timestamp_to_frame(instance()->current_frame_timestamp);
    if (!stage_with_frame)
        return;
    param.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = (Tp)value;
}

class STELLA_VSLAM_API metrics_copy {
public:
    metrics_copy(metrics const& m);
    metrics const& operator()();
protected:
    metrics m_copy;
};

STELLA_VSLAM_API void metrics_html_test(std::string const& directory, std::array<std::string, 3> image_filenames);

} // namespace stella_vslam_bfx
