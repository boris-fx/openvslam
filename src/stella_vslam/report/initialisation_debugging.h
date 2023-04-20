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
#include <list>
#include <vector>

#include "stella_vslam/exports.h"

namespace stella_vslam_bfx {

template<typename T>
struct frame_param
{
    std::map<std::array<double, 2>, T> by_timestamp; // Map from a pair of frame identified by timestamp, to some data
    std::map<std::array<int,    2>, T> by_frame;     // Map from a pair of frame identified by frame number, to some data
};

// This is for debugging the initialisation stage of stella - especially when focal length is unknown
class STELLA_VSLAM_API initialisation_debugging {
public:

    initialisation_debugging();

    std::array<double, 2> current_init_frames; 
    double video_width;
    
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

    void submit_fundamental_decomp_debugging(double error_for_max_focal_length,
                                             double min_error,
                                             double best_focal_length,
                                             double de_df_plus,
                                             double de_df_minus,
                                             double min_error_percent_max_focal_error,
                                             std::map<double, double> focal_length_to_error);

    void submit_epipolar_estimator_debugging(double min_error, double initial_focal_length, double best_focal_length,
        double dedf, double dedtu, double dedtv, double dedrx, double dedry, double dedrz,
        double sd_dedf, double sd_dedtu, double sd_dedtv, double sd_dedrx, double sd_dedry, double sd_dedrz);

    void submit_feature_motions(double quantile_25, double quantile_50, double quantile_75);

    std::map<double, int> feature_count_by_timestamp;

    void create_frame_data(std::map<double, int> const& timestamp_to_video_frame);

    void add_to_html(std::stringstream& html, std::optional<double> ground_truth_focal_length_x_pixels) const;
    void save_html_report(std::string_view const& filename, std::optional<double> ground_truth_focal_length_x_pixels) const;

    std::optional<int> average_init_frame_feature_count() const;
    std::optional<int> average_init_frame_unguided_match_count() const;

protected:
    
    std::map<int, int> feature_count_by_frame;

    frame_param<double> p_num_matches;

    frame_param<double> p_parallax;
    frame_param<double> p_cost_H;
    frame_param<double> p_cost_F;

    frame_param<double> p_error_for_max_focal_length;
    frame_param<double> p_min_error;
    frame_param<double> p_best_focal_length;
    frame_param<double> p_best_focal_length_bisection;
    frame_param<double> p_de_df_plus;
    frame_param<double> p_de_df_minus;
    frame_param<double> p_min_error_percent_max_focal_error;

    frame_param<std::map<double, double>> p_focal_length_to_error;
    frame_param<std::map<double, double>> p_fov_to_error;
    
    frame_param<double> p_dec_error_for_max_focal_length;
    frame_param<double> p_dec_min_error;
    frame_param<double> p_dec_best_focal_length;
    frame_param<double> p_dec_de_df_plus;
    frame_param<double> p_dec_de_df_minus;
    frame_param<double> p_dec_min_error_percent_max_focal_error;
    frame_param<std::map<double, double>> p_dec_focal_length_to_error;

    frame_param<double> p_ep_min_error;
    frame_param<double> p_ep_initial_focal_length;
    frame_param<double> p_ep_best_focal_length;
    frame_param<double> p_ep_dedf;
    frame_param<double> p_ep_dedtu;
    frame_param<double> p_ep_dedtv;
    frame_param<double> p_ep_dedrx;
    frame_param<double> p_ep_dedry;
    frame_param<double> p_ep_dedrz;
    frame_param<double> p_ep_sd_dedf;
    frame_param<double> p_ep_sd_dedtu;
    frame_param<double> p_ep_sd_dedtv;
    frame_param<double> p_ep_sd_dedrx;
    frame_param<double> p_ep_sd_dedry;
    frame_param<double> p_ep_sd_dedrz;

    frame_param<double> feature_motion_quantile_25;
    frame_param<double> feature_motion_quantile_50;
    frame_param<double> feature_motion_quantile_75;

    std::list<frame_param<double>*> frame_params_double();
    std::list<frame_param<std::map<double, double>>*> frame_params_map_double_double();

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

struct scalar_measurement {
    scalar_measurement(double value = -1.0, double variance = -1.0)
        : value(value), variance(variance) {}
    double value;
    double variance;
};

scalar_measurement combine_measurements(std::vector<scalar_measurement> const& measurements);

} // namespace stella_vslam_bfx