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
    void submit_mapping_reset(double timestamp);

    double current_frame_timestamp;

    // Convert the timestamped metrics to frame numbers
    void create_frame_metrics();

    int total_frames() const;
    std::set<std::pair<std::string, tracking_problem_level>> problems() const;
    std::optional<tracking_problem_level> max_problem_level() const;
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
        //double timestamp;
        //int frame; // set in create_frame_metrics
        stage_and_frame stage_with_frame; // tracking stage and frame
    };

    //template<typename T>
    //struct frame_param
    //{
    //    std::map<double, T> by_timestamp; // Map from a pair of frame identified by timestamp, to some data
    //    std::map<int, T> by_frame;     // Map from a pair of frame identified by frame number, to some data
    //    std::map<double, double> graph() const;
    //};

    template<typename T>
    struct stage_and_frame_param {
        std::array<std::map<int, T>, max_stage> by_stage_and_frame;
        std::list<curve_section> graph() const;
    };
protected:

    std::list<focal_estimate> intermediate_focal_estimates;

    stage_and_frame_param<unsigned int> map_size;
    stage_and_frame_param<unsigned int> tracking_fail_count;

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
