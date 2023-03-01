/** \file
 * \brief Camera tracking metrics recorded during and ater tracking
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <string>
#include <map>
#include <thread>
#include <array>
#include <optional>

#include "stella_vslam/exports.h"

namespace stella_vslam_bfx {

// This is for debugging the initialisation stage of stella - especially when focal length is unknown
class STELLA_VSLAM_API initialisation_debugging {
public:

    initialisation_debugging();

    std::array<double, 2> current_init_frames; 
    std::map<double, int> feature_count_by_timestamp;

    bool active() const;

    void submit_feature_match_debugging(unsigned int num_matches);
    void submit_parallax_debugging(double parallax);
    void submit_homography_fundamental_cost(double cost_H, double cost_F);
    void submit_fundamental_to_focal_length_debugging(double error_for_max_focal_length,
                                                      double min_error,
                                                      double best_focal_length,
                                                      double best_focal_length_bisection,
                                                      double de_df_plus,
                                                      double de_df_minus,
                                                      double min_error_percent_max_focal_error,
                                                      std::map<double, double> focal_length_to_error,
                                                      std::map<double, double> fov_to_error);

    void create_frame_data(std::map<double, int> const& timestamp_to_video_frame);

    std::map<std::array<int, 2>, double> frame_to_num_matches;
    std::map<std::array<int, 2>, double> frame_to_parallax;
    std::map<std::array<int, 2>, double> frame_to_cost_H;
    std::map<std::array<int, 2>, double> frame_to_cost_F;
    std::map<std::array<int, 2>, double> frame_to_error_for_max_focal_length;
    std::map<std::array<int, 2>, double> frame_to_min_error;
    std::map<std::array<int, 2>, double> frame_to_best_focal_length;
    std::map<std::array<int, 2>, double> frame_to_best_focal_length_bisection;
    std::map<std::array<int, 2>, double> frame_to_de_df_plus;
    std::map<std::array<int, 2>, double> frame_to_de_df_minus;
    std::map<std::array<int, 2>, double> frame_to_min_error_percent_max_focal_error;
    std::map<std::array<int, 2>, std::map<double, double>> frame_to_focal_length_to_error;
    std::map<std::array<int, 2>, std::map<double, double>> frame_to_fov_to_error;

    void add_to_html(std::stringstream& html, std::optional<double> ground_truth_focal_length_x_pixels) const;
    void save_html_report(std::string_view const& filename, std::optional<double> ground_truth_focal_length_x_pixels) const;

    std::map<int, int> feature_count_by_frame;

protected:
   
    std::map<std::array<double, 2>, double> timestamp_to_num_matches;
    std::map<std::array<double, 2>, double> timestamp_to_parallax;
    std::map<std::array<double, 2>, double> timestamp_to_cost_H;
    std::map<std::array<double, 2>, double> timestamp_to_cost_F;
    std::map<std::array<double, 2>, double> timestamp_to_error_for_max_focal_length;
    std::map<std::array<double, 2>, double> timestamp_to_min_error;
    std::map<std::array<double, 2>, double> timestamp_to_best_focal_length;
    std::map<std::array<double, 2>, double> timestamp_to_best_focal_length_bisection;
    std::map<std::array<double, 2>, double> timestamp_to_de_df_plus;
    std::map<std::array<double, 2>, double> timestamp_to_de_df_minus;
    std::map<std::array<double, 2>, double> timestamp_to_min_error_percent_max_focal_error;
    std::map<std::array<double, 2>, std::map<double, double>> timestamp_to_focal_length_to_error;
    std::map<std::array<double, 2>, std::map<double, double>> timestamp_to_fov_to_error;
    
    friend class metrics;
    bool is_active;
};

// This is for assigning a name to each thread which can be printed during debugging. It should be somewhere else, and may be temporary.
class thread_dubugging {
private:
    inline static thread_dubugging* instance{nullptr};
    thread_dubugging() = default;
    ~thread_dubugging() = default;

    std::map<std::thread::id, std::string> thread_id_to_name; // protect with mutex

public:

    std::string thread_name() const;
    void set_thread_name(std::string name);

    thread_dubugging(const thread_dubugging&) = delete;
    thread_dubugging& operator=(const thread_dubugging&) = delete;

    static thread_dubugging* get_instance();
};

} // namespace stella_vslam_bfx