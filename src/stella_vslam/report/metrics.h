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

    template<typename T>
    struct stage_and_frame_param {
        std::array<std::map<int, T>, max_stage> by_stage_and_frame;
        std::list<curve_section> graph() const;
    };

    template<typename T>
    struct stage_and_frame_pair_param {
        std::array<std::map<std::pair<int, int>, T>, max_stage> by_stage_and_frame;
        std::list<curve_section> graph() const;
        std::list<curve_section> frame_separation_graph() const;
    };

protected:

    void add_matching_details(std::stringstream& html, curve_section const& graph_feature_count, curve_section const& graph_num_matches) const;

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

    std::list<double> mapping_reset_timestamps;
    std::list<int> mapping_reset_frames;

    friend class metrics_copy;

    initialisation_debugging initialisation_debug_object;

    inline static metrics* instance{nullptr};
    metrics() = default;
    ~metrics() = default;
    metrics(const metrics&) = default;
};

class STELLA_VSLAM_API metrics_copy {
public:
    metrics_copy(metrics const& m);
    metrics const& operator()();
protected:
    metrics m_copy;
};

STELLA_VSLAM_API void metrics_html_test(std::string const& directory, std::array<std::string, 3> image_filenames);

} // namespace stella_vslam_bfx
