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
#include <set>

#include "stella_vslam/exports.h"

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

    std::optional<double> knownFocalLengthXPixels;       // Can be used during tracking if set

    std::optional<double> groundTruthFocalLengthXPixels; // Not used in tracking, but in evaluation
    std::optional<double> groundTruthFilmBackWidthMM;    // Not used in tracking, but in evaluation
    std::optional<double> groundTruthFilmBackHeightMM;   // Not used in tracking, but in evaluation
    std::optional<double> groundTruthFocalLengthMM;      // Not used in tracking, but in evaluation

    //
    std::optional<double> ground_truth_pixel_aspect_ratio() const;
    std::optional<double> ground_truth_focal_length_x_pixels() const;
    std::optional<double> calculated_focal_length_mm(double calculated_focal_length_x_pixels) const;


};

struct STELLA_VSLAM_API timings {

    double forward_mapping;
    double backward_mapping;
    double loop_closing;
    double optimisation; // aka. bundle adjust
    double tracking; // aka. resection

    double total_time_sec() const;
};

enum tracking_problem_level {
    tracking_problem_warning,
    tracking_problem_fatal
};

/**
 * \brief Metrics for the tracking of one video section
 */
class STELLA_VSLAM_API metrics {
public:

    // todo: add a multithreaded access mutex

    video_metadata input_video_metadata;

    timings track_timings;

    double calculated_focal_length_x_pixels;

    int solved_frame_count;
    int unsolved_frame_count;
    int num_points;

    std::set<int> initialisation_frames;

    int total_frames() const;
    std::set<std::pair<std::string, tracking_problem_level>> problems() const;
    std::optional<tracking_problem_level> max_problem_level() const;
public:
    std::set<double> initialisation_frame_timestamps; /// For tracker internal use

    static const double par_percent_error_trigger;
    static const double focal_percent_error_trigger;
    
    metrics& operator=(const metrics&) = delete;

    static metrics* get_instance();
    void clear(); // clear for a new camera track

    void save_html_report(std::string_view const& filename, std::string thumbnail_path) const;
    
    struct track_test_info {
        metrics const* m;
        std::string thumbnail_filename; // thumbnail image
        std::string html_filename; // html for the track
    };
    static void save_html_overview(std::string_view const& filename,
                                   std::list<track_test_info> const& track_test_info_list);

protected:
    friend class metrics_copy;

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