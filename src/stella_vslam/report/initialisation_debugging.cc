#include "initialisation_debugging.h"

#include <vector>
#include <cmath>
#include <numeric>
#define _USE_MATH_DEFINES
#include <math.h>

#include "plot_html.h"
#include <stella_vslam/solve/fundamental_consistency.h>

namespace stella_vslam_bfx {

scalar_measurement combine_measurements(std::vector<scalar_measurement> const& measurements) {
    double sum_of_reciprocal_variance(0);
    for (auto const& m : measurements)
        sum_of_reciprocal_variance += 1.0 / m.variance;

    double weighted_mean(0);
    for (auto const& m : measurements)
        weighted_mean += m.value / m.variance;
    weighted_mean /= sum_of_reciprocal_variance;

    return scalar_measurement(weighted_mean, 1.0 / sum_of_reciprocal_variance);
}

// uncertainty is in the units of the measurenent, like standard devaition
void cumulative_stochastic_estimate(std::map<double, double> const& measurement,
                                    std::map<double, double> const& measurement_uncertainty,
                                    std::map<double, double>      & estimate,
                                    std::map<double, double>      & estimate_uncertainty)
{
    std::vector<double> keys;
    std::vector<scalar_measurement> all_measurements;
    std::vector<scalar_measurement> estimates;
    for (auto const& m : measurement) {
        auto f = measurement_uncertainty.find(m.first);
        if (f != measurement_uncertainty.end()) {
            keys.push_back(m.first);
            all_measurements.push_back({m.second, f->second * f->second});
            estimates.push_back(combine_measurements(all_measurements));
        }
    }
    estimate.clear();
    estimate_uncertainty.clear();
    for (int i = 0; i < keys.size(); ++i) {
        estimate[keys[i]] = estimates[i].value;
        estimate_uncertainty[keys[i]] = sqrt(estimates[i].variance);
    }
}

double fx(std::map<double, double> const& fx_discrete, double x) {
    if (fx_discrete.empty())
        return -1;
    auto upper = fx_discrete.upper_bound(x);
    if (upper == fx_discrete.begin())
        return fx_discrete.begin()->second;
    auto lower = std::prev(upper);

    double frac = (x - lower->first) / (upper->first - lower->first);
    return naive_lerp(lower->second, upper->second, frac);
}

std::map<std::array<int, 2>, double> error_for_focal_length(double ground_truth_focal_length_x_pixels,
                                           std::map<std::array<int, 2>, std::map<double, double>> const& frame_to_focal_length_to_error) {
    std::map<std::array<int, 2>, double> result;
    for (auto const& i : frame_to_focal_length_to_error)
        result[i.first] = fx(i.second, ground_truth_focal_length_x_pixels);
    return result;
}

initialisation_debugging::initialisation_debugging()
: video_width(-1), current_init_frames({-1,-1})
{
}

bool initialisation_debugging::active() const {
    return is_active;
}

void initialisation_debugging::submit_feature_match_debugging(unsigned int num_matches)
{
    // a general metric
    //if (!is_active)
    //    return;

    p_num_matches.by_timestamp[current_init_frames] = double(num_matches);
}

void initialisation_debugging::submit_parallax_debugging(double parallax)
{
    if (!is_active)
        return;

    p_parallax.by_timestamp[current_init_frames] = parallax;
}

void initialisation_debugging::submit_homography_fundamental_cost(double cost_H, double cost_F)
{
    if (!is_active)
        return;

    p_cost_H.by_timestamp[current_init_frames] = cost_H;
    p_cost_F.by_timestamp[current_init_frames] = cost_F;
}

void initialisation_debugging::submit_fundamental_to_focal_length_debugging(double error_for_max_focal_length,
                                                                            double min_error,
                                                                            double best_focal_length,
                                                                            double best_focal_length_bisection,
                                                                            double de_df_plus,
                                                                            double de_df_minus,
                                                                            double min_error_percent_max_focal_error,
                                                                            std::map<double, double> focal_length_to_error,
                                                                            std::map<double, double> fov_to_error)
{
    if (!is_active)
        return;

    p_error_for_max_focal_length.by_timestamp[current_init_frames] = error_for_max_focal_length;
    p_min_error.by_timestamp[current_init_frames] = min_error;
    p_best_focal_length.by_timestamp[current_init_frames] = best_focal_length;
    p_best_focal_length_bisection.by_timestamp[current_init_frames] = best_focal_length_bisection;
    p_de_df_plus.by_timestamp[current_init_frames] = de_df_plus;
    p_de_df_minus.by_timestamp[current_init_frames] = de_df_minus;
    p_min_error_percent_max_focal_error.by_timestamp[current_init_frames] = min_error_percent_max_focal_error;

    p_focal_length_to_error.by_timestamp[current_init_frames] = focal_length_to_error;
    p_fov_to_error.by_timestamp[current_init_frames] = fov_to_error;
}

void initialisation_debugging::submit_fundamental_decomp_debugging(double error_for_max_focal_length,
    double min_error,
    double best_focal_length,
    double de_df_plus,
    double de_df_minus,
    double min_error_percent_max_focal_error,
    std::map<double, double> focal_length_to_error)
{
    if (!is_active)
        return;

    p_dec_error_for_max_focal_length.by_timestamp[current_init_frames] = error_for_max_focal_length;
    p_dec_min_error.by_timestamp[current_init_frames] = min_error;
    p_dec_best_focal_length.by_timestamp[current_init_frames] = best_focal_length;
    p_dec_de_df_plus.by_timestamp[current_init_frames] = de_df_plus;
    p_dec_de_df_minus.by_timestamp[current_init_frames] = de_df_minus;
    p_dec_min_error_percent_max_focal_error.by_timestamp[current_init_frames] = min_error_percent_max_focal_error;

    p_dec_focal_length_to_error.by_timestamp[current_init_frames] = focal_length_to_error;

}

void initialisation_debugging::submit_epipolar_estimator_debugging(double min_error, double initial_focal_length, double best_focal_length,
    double dedf, double dedtu, double dedtv, double dedrx, double dedry, double dedrz,
    double sd_dedf, double sd_dedtu, double sd_dedtv, double sd_dedrx, double sd_dedry, double sd_dedrz)
{
    if (!is_active)
        return;

    p_ep_min_error.by_timestamp[current_init_frames] = min_error;
    p_ep_initial_focal_length.by_timestamp[current_init_frames] = initial_focal_length;
    p_ep_best_focal_length.by_timestamp[current_init_frames] = best_focal_length;

    p_ep_dedf.by_timestamp[current_init_frames] = dedf;
    p_ep_dedtu.by_timestamp[current_init_frames] = dedtu;
    p_ep_dedtv.by_timestamp[current_init_frames] = dedtv;
    p_ep_dedrx.by_timestamp[current_init_frames] = dedrx;
    p_ep_dedry.by_timestamp[current_init_frames] = dedry;
    p_ep_dedrz.by_timestamp[current_init_frames] = dedrz;

    p_ep_sd_dedf.by_timestamp[current_init_frames] = sd_dedf;
    p_ep_sd_dedtu.by_timestamp[current_init_frames] = sd_dedtu;
    p_ep_sd_dedtv.by_timestamp[current_init_frames] = sd_dedtv;
    p_ep_sd_dedrx.by_timestamp[current_init_frames] = sd_dedrx;
    p_ep_sd_dedry.by_timestamp[current_init_frames] = sd_dedry;
    p_ep_sd_dedrz.by_timestamp[current_init_frames] = sd_dedrz;
}

void initialisation_debugging::submit_feature_motions(double quantile_25, double quantile_50, double quantile_75)
{
    if (!is_active)
        return;

    feature_motion_quantile_25.by_timestamp[current_init_frames] = quantile_25;
    feature_motion_quantile_50.by_timestamp[current_init_frames] = quantile_50;
    feature_motion_quantile_75.by_timestamp[current_init_frames] = quantile_75;
}

template<typename T>
void transform_frame_data(std::map<std::array<double, 2>, T> const& input,
                          std::map<std::array<int, 2>, T>& output,
                          std::map<double, int> const& index_transform)
{
    output.clear();
    for (auto const& i : input) {
        auto f0 = index_transform.find(i.first[0]);
        auto f1 = index_transform.find(i.first[1]);
        if (f0 != index_transform.end() && f1 != index_transform.end())
            output[{f0->second, f1->second}] = i.second;
    }
}

template<typename T>
void transform_frame_data(std::map<double, T> const& input,
                          std::map<int, T>& output,
                          std::map<double, int> const& index_transform) {
    output.clear();
    for (auto const& i : input) {
        auto f = index_transform.find(i.first);
        if (f != index_transform.end())
            output[f->second] = i.second;
    }
}

void initialisation_debugging::create_frame_data(std::map<double, int> const& timestamp_to_video_frame)
{
    std::list<frame_param<double>*> p_double = frame_params_double();
    for (auto &param : p_double)
        transform_frame_data(param->by_timestamp, param->by_frame, timestamp_to_video_frame);

    std::list<frame_param<std::map<double, double>>*> p_map_double_double = frame_params_map_double_double();
    for (auto& param : p_map_double_double)
        transform_frame_data(param->by_timestamp, param->by_frame, timestamp_to_video_frame);

    transform_frame_data(feature_count_by_timestamp, feature_count_by_frame, timestamp_to_video_frame);
}

std::list<frame_param<double>*> initialisation_debugging::frame_params_double()
{
    std::list<frame_param<double>*> result;

    result.push_back(&p_num_matches);

    result.push_back(&p_parallax);
    result.push_back(&p_cost_H);
    result.push_back(&p_cost_F);

    result.push_back(&p_error_for_max_focal_length);
    result.push_back(&p_min_error);
    result.push_back(&p_best_focal_length);
    result.push_back(&p_best_focal_length_bisection);
    result.push_back(&p_de_df_plus);
    result.push_back(&p_de_df_minus);
    result.push_back(&p_min_error_percent_max_focal_error);

    result.push_back(&p_dec_error_for_max_focal_length);
    result.push_back(&p_dec_min_error);
    result.push_back(&p_dec_best_focal_length);
    result.push_back(&p_dec_de_df_plus);
    result.push_back(&p_dec_de_df_minus);
    result.push_back(&p_dec_min_error_percent_max_focal_error);

    result.push_back(&p_ep_min_error);
    result.push_back(&p_ep_initial_focal_length);
    result.push_back(&p_ep_best_focal_length);

    result.push_back(&p_ep_dedf);
    result.push_back(&p_ep_dedtu);
    result.push_back(&p_ep_dedtv);
    result.push_back(&p_ep_dedrx);
    result.push_back(&p_ep_dedry);
    result.push_back(&p_ep_dedrz);

    result.push_back(&p_ep_sd_dedf);
    result.push_back(&p_ep_sd_dedtu);
    result.push_back(&p_ep_sd_dedtv);
    result.push_back(&p_ep_sd_dedrx);
    result.push_back(&p_ep_sd_dedry);
    result.push_back(&p_ep_sd_dedrz);

    result.push_back(&feature_motion_quantile_25);
    result.push_back(&feature_motion_quantile_50);
    result.push_back(&feature_motion_quantile_75);

    return result;
}

std::list<frame_param<std::map<double, double>>*> initialisation_debugging::frame_params_map_double_double()
{
    std::list<frame_param<std::map<double, double>>*> result;

    result.push_back(&p_focal_length_to_error);
    result.push_back(&p_fov_to_error);
    result.push_back(&p_dec_focal_length_to_error);

    return result;
}

// map entry {a, b}->c becomes b->c
template<typename T>
std::map<double, T> select_second_frame_data(std::map<std::array<int, 2>, T> const& input)
{
    std::map<double, T> output;
    int first_frame = input.empty() ? 0 : input.begin()->first[0];
    for (auto const& i : input)
        if (i.first[0]==first_frame)
            output[i.first[1]] = i.second;
    return output;
}

// map entry {a, b}->c becomes b->c
template<typename T>
std::map<double, T> select_second_frame_data(frame_param<T> const& input)
{
    return select_second_frame_data(input.by_frame);
}

// map entry {a, b}->c becomes b->c*scale
template<typename T>
std::map<double, T> select_scaled_second_frame_data(std::map<std::array<int, 2>, T> const& input, double scale) {
    std::map<double, T> output;
    int first_frame = input.empty() ? 0 : input.begin()->first[0];
    for (auto const& i : input)
        if (i.first[0] == first_frame)
            output[i.first[1]] = i.second * scale;
    return output;
}

std::map<double, double> percent_deviation_from_value(std::map<double, double> const& data, double value)
{
    std::map<double, double> output;
    for (auto const& key : data)
        output[key.first] = 100.0 * fabs((key.second - value) / value);
    return output;
}

std::map<double, double> percent_deviation_from_value(std::map<double, double> const& data, std::map<double, double> const& value) {
    std::map<double, double> output;
    for (auto const& key : data) {
        auto f = value.find(key.first);
        if (f != value.end())
            output[key.first] = 100.0 * fabs((key.second - f->second) / f->second);
    }
    return output;
}

std::map<double, double> percent_of_value(std::map<double, double> const& data, std::map<double, double> const& value) {
    std::map<double, double> output;
    for (auto const& key : data) {
        auto f = value.find(key.first);
        if (f != value.end())
            output[key.first] = 100.0 * key.second / f->second;
    }
    return output;
}

std::map<double, double> multiply(std::map<double, double> const& data_1, std::map<double, double> const& data_2, double scalar_multiplier) {
    std::map<double, double> output;
    for (auto const& d_1 : data_1) {
        auto d_2 = data_2.find(d_1.first);
        if (d_2 != data_2.end())
            output[d_1.first] = scalar_multiplier * d_1.second * d_2->second;
    }
    return output;
}

std::map<double, double> rescale_graph(std::map<double, double> const& data, double scale)
{
    std::map<double, double> output;
    for (auto const& d : data)
        output[d.first] = scale * d.second;
    return output;
}

std::map<double, double> constant_value_graph(std::map<double, double> const& data, double value) {
    std::map<double, double> output;
    if (!data.empty()) {
        output[data.begin()->first] = value;
        output[data.rbegin()->first] = value;
    }
    return output;
}

std::map<double, double> focal_length_x_to_FOV_graph(std::map<double, double> const& data, double image_width)
{
    std::map<double, double> output;
    for (auto const& d : data)
        output[2.0 * atan2(0.5 * image_width, d.first) * 180.0 / M_PI] = d.second;
    return output;
}

void initialisation_debugging::add_to_html(std::stringstream& html, std::optional<double> ground_truth_focal_length_x_pixels) const {
    if (!active())
        return;


    std::map<double, double> graph_num_matches = select_second_frame_data(p_num_matches);
    std::map<double, double> graph_parallax = select_second_frame_data(p_parallax);
    std::map<double, double> graph_cost_H = select_second_frame_data(p_cost_H);
    std::map<double, double> graph_cost_F = select_second_frame_data(p_cost_F);
    std::map<double, double> graph_error_for_max_focal_length = select_second_frame_data(p_error_for_max_focal_length);
    std::map<double, double> graph_min_error = select_second_frame_data(p_min_error);
    std::map<double, double> graph_de_df_plus = select_second_frame_data(p_de_df_plus);
    std::map<double, double> graph_de_df_minus = select_second_frame_data(p_de_df_minus);
    std::map<double, double> graph_min_error_percent_max_focal_error = select_second_frame_data(p_min_error_percent_max_focal_error);
    std::map<double, double> graph_best_focal_length = select_second_frame_data(p_best_focal_length);
    std::map<double, double> graph_best_focal_length_bisection = select_second_frame_data(p_best_focal_length_bisection);

    std::map<double, double> dec_graph_best_focal_length = select_second_frame_data(p_dec_best_focal_length);

    std::map<double, double> graph_ep_min_error = select_second_frame_data(p_ep_min_error);
    std::map<double, double> graph_ep_initial_focal_length = select_second_frame_data(p_ep_initial_focal_length);
    std::map<double, double> graph_ep_best_focal_length = select_second_frame_data(p_ep_best_focal_length);

    std::map<double, double> graph_ep_dedf = select_second_frame_data(p_ep_dedf);
    std::map<double, double> graph_ep_dedtu = select_second_frame_data(p_ep_dedtu);
    std::map<double, double> graph_ep_dedtv = select_second_frame_data(p_ep_dedtv);
    std::map<double, double> graph_ep_dedrx = select_second_frame_data(p_ep_dedrx);
    std::map<double, double> graph_ep_dedry = select_second_frame_data(p_ep_dedry);
    std::map<double, double> graph_ep_dedrz = select_second_frame_data(p_ep_dedrz);

    std::map<double, double> graph_ep_sd_dedf = select_second_frame_data(p_ep_sd_dedf);
    std::map<double, double> graph_ep_sd_dedtu = select_second_frame_data(p_ep_sd_dedtu);
    std::map<double, double> graph_ep_sd_dedtv = select_second_frame_data(p_ep_sd_dedtv);
    std::map<double, double> graph_ep_sd_dedrx = select_second_frame_data(p_ep_sd_dedrx);
    std::map<double, double> graph_ep_sd_dedry = select_second_frame_data(p_ep_sd_dedry);
    std::map<double, double> graph_ep_sd_dedrz = select_second_frame_data(p_ep_sd_dedrz);

    std::map<double, double> graph_feature_motion_quantile_25 = select_second_frame_data(feature_motion_quantile_25);
    std::map<double, double> graph_feature_motion_quantile_50 = select_second_frame_data(feature_motion_quantile_50);
    std::map<double, double> graph_feature_motion_quantile_75 = select_second_frame_data(feature_motion_quantile_75);

    std::map<double, double> graph_error_at_ground_truth_focal_length;
    std::map<double, double> graph_gt_error_percent_max_focal_error;
    std::map<double, double> graph_min_error_percent_gt_error;
    if (ground_truth_focal_length_x_pixels) {
        std::map<std::array<int, 2>, double> frame_to_error_at_ground_truth_focal_length
            = error_for_focal_length(ground_truth_focal_length_x_pixels.value(),
                                     p_focal_length_to_error.by_frame);
        graph_error_at_ground_truth_focal_length = select_second_frame_data(frame_to_error_at_ground_truth_focal_length);
        graph_gt_error_percent_max_focal_error = percent_of_value(graph_error_at_ground_truth_focal_length, graph_error_for_max_focal_length);
        graph_min_error_percent_gt_error = percent_deviation_from_value(graph_min_error, graph_error_at_ground_truth_focal_length);
    
    }

    std::map<double, double> graph_percent_matches;
    if (!p_num_matches.by_frame.empty()) {
        auto f = feature_count_by_frame.find(p_num_matches.by_frame.begin()->first[0]);
        if (f != feature_count_by_frame.end()) {
            double scale = 100.0 / double(f->second);
            graph_percent_matches = select_scaled_second_frame_data(p_num_matches.by_frame, scale);

        }
    }
    
    //std::optional<double> percent_max(110);
    axis_scaling percent_y(110);
//    std::optional<double> focal_max(ground_truth_focal_length_x_pixels ? std::optional<double>(2.0 * ground_truth_focal_length_x_pixels.value()) : std::nullopt);
    axis_scaling focal_y = ground_truth_focal_length_x_pixels ? axis_scaling(2.0 * ground_truth_focal_length_x_pixels.value()) : range_behaviour::no_max;
    range_behaviour full_x(range_behaviour::no_max), full_y(range_behaviour::no_max);
    range_behaviour median_y(range_behaviour::max_from_median);
    std::optional<double> no_gt, focal_gt(ground_truth_focal_length_x_pixels ? std::optional<double>(ground_truth_focal_length_x_pixels.value()) : std::nullopt);

    axis_scaling focal_x = ground_truth_focal_length_x_pixels ? axis_scaling(3.0 * ground_truth_focal_length_x_pixels.value()) : range_behaviour::no_max;

    html << "<h2>Feature Points and Matching</h2>" << std::endl;

    write_graph_as_svg(html, Graph("Second init frame", "Feature motion (pixels)", std::set<Curve>({ {"First Quartile", graph_feature_motion_quantile_25},
                                                                                                               {"Second Quartile", graph_feature_motion_quantile_50},
                                                                                                               {"Third Quartile", graph_feature_motion_quantile_75} }), full_x, full_y, no_gt));
    write_graph_as_svg(html, Graph("Second init frame", "Num feature matches", std::set<Curve>({ {"Match count", graph_num_matches} }), full_x, full_y, no_gt));
    html << "<hr>" << std::endl;

    html << "<h2>Structure type</h2>" << std::endl;

    write_graph_as_svg(html, Graph("Second init frame", "Cost", std::set<Curve>({ {"H cost", graph_cost_H}, {"F cost", graph_cost_F} }), full_x, full_y, no_gt));
    html << "<p>Average feature match deviation from the geometric model (pixels - with max of a few pixels). H cost is deviation from a planar scene model, F-cost measures deviation from a non-planar scene model.</p>" << std::endl;
    html << "<hr>" << std::endl;

    html << "<h2>Focal length from F-matrix (non-planar scene model)</h2>" << std::endl;


//    write_graph_as_svg(html, Graph("Second init frame", "Epipolar focal length", std::set<Curve>({ {"Focal length est.", graph_ep_best_focal_length}, {"Initial focal length", graph_ep_initial_focal_length}}), full_x, focal_y, focal_gt));
  //  write_graph_as_svg(html, Graph("Second init frame", "Best Focal Length", std::set<Curve>({ {"Focal length", graph_best_focal_length}, {"Focal length bisection", graph_best_focal_length_bisection}, {"Dec Focal length", dec_graph_best_focal_length} }), full_x, focal_y, focal_gt));

    auto graph_singular_value(graph_ep_initial_focal_length.empty() ? graph_best_focal_length_bisection : graph_ep_initial_focal_length);
    write_graph_as_svg(html, Graph("Second init frame", "Estimated Focal Length", std::set<Curve>({ {"Optimised", graph_ep_best_focal_length},
                                                                                                    {"Singular value", graph_singular_value},
                                                                                                    {"Decomposition-recomp.", dec_graph_best_focal_length} }), full_x, focal_y, focal_gt));


    // Simple stochastic analysis
    {
        std::map<double, double> const& measurement(graph_best_focal_length_bisection);
        double uncertainty_multiplier(10.0 / 100.0);
        std::map<double, double> measurement_uncertainty = multiply(graph_min_error_percent_max_focal_error, measurement, uncertainty_multiplier);

        std::map<double, double> estimate, estimate_uncertainty;
        cumulative_stochastic_estimate(measurement, measurement_uncertainty, estimate, estimate_uncertainty);

        write_graph_as_svg(html, Graph("Second init frame", "Stochastic estimates", std::set<Curve>({ {"measurement", measurement}, {"estimate", estimate} }), full_x, focal_y, focal_gt));
        write_graph_as_svg(html, Graph("Second init frame", "Stochastic uncertainty", std::set<Curve>({ {"measurement uncertainty", measurement_uncertainty}, {"estimate uncertainty", estimate_uncertainty} }), full_x, focal_y, no_gt));
    }

    html << "<hr>" << std::endl;

    html << "<h2>Optimised F-matrix deviation</h2>" << std::endl;

    write_graph_as_svg(html, Graph("Second init frame", "Epipolar pixel error", std::set<Curve>({ {"Error", graph_ep_min_error} }), full_x, full_y, no_gt));

    html << "<p>Average feature match deviation from optimised non-planar geometric model (pixels - with no max).</p>" << std::endl;
   
    html << "<hr>" << std::endl;

    write_graph_as_svg(html, Graph("Second init frame", "de/df", std::set<Curve>({ {"de/df", graph_ep_dedf}, {"sd de/df", graph_ep_sd_dedf} }), full_x, median_y, no_gt));
    write_graph_as_svg(html, Graph("Second init frame", "de/dt", std::set<Curve>({ {"de/dtu", graph_ep_dedtu}, {"de/dtv", graph_ep_dedtv}, {"sd de/dtu", graph_ep_sd_dedtu}, {"sd de/dtv", graph_ep_sd_dedtv} }), full_x, median_y, no_gt));
    write_graph_as_svg(html, Graph("Second init frame", "de/dr", std::set<Curve>({ {"de/drx", graph_ep_dedrx},  {"de/dry", graph_ep_dedry}, {"de/drz", graph_ep_dedrz},
                                                                                             {"sd de/drx", graph_ep_sd_dedrx},  {"sd de/dry", graph_ep_sd_dedry}, {"sd de/drz", graph_ep_sd_dedrz} }), full_x, median_y, no_gt));

    if (ground_truth_focal_length_x_pixels) {
        std::map<double, double> graph_focal_deviation = percent_deviation_from_value(graph_best_focal_length_bisection, ground_truth_focal_length_x_pixels.value());
        std::set<Curve> curves({{"% Error in focal est.", graph_focal_deviation}, {"% Comp. error measure", graph_min_error_percent_max_focal_error}});
        if (!graph_percent_matches.empty())
            curves.insert({"Feature match %", graph_percent_matches});
        if (!graph_error_at_ground_truth_focal_length.empty()) {
            std::map<double, double> graph_focal_deviation_2 = percent_deviation_from_value(graph_error_at_ground_truth_focal_length, graph_min_error);
            curves.insert({"min error/gt error", graph_focal_deviation_2});
        }
        write_graph_as_svg(html, Graph("Second init frame", "Error Percent", curves, full_x, percent_y, no_gt));
    }
    write_graph_as_svg(html, Graph("Second init frame", "Focal Confidence", std::set<Curve>({{"dE/dF(+)", graph_de_df_plus}, {"-dE/dF(-)", graph_de_df_minus}}), full_x, full_y, no_gt));
    write_graph_as_svg(html, Graph("Second init frame", "Parallax", std::set<Curve>({{"Parallax", graph_parallax}}), full_x, full_y, no_gt));
    write_graph_as_svg(html, Graph("Second init frame", "Error at tiny focal length", std::set<Curve>({{"Error", graph_error_for_max_focal_length}}), full_x, full_y, no_gt));
    if (!graph_error_at_ground_truth_focal_length.empty()) {
        write_graph_as_svg(html, Graph("Second init frame", "Geometric Error Percent", std::set<Curve>({{"% error", graph_min_error_percent_max_focal_error}, {"% gt error", graph_gt_error_percent_max_focal_error}, {"min % gt error",  graph_min_error_percent_gt_error}}), full_x, full_y, no_gt));
        write_graph_as_svg(html, Graph("Second init frame", "Min Geometric Error", std::set<Curve>({{"Error", graph_min_error}, {"Error @gt", graph_error_at_ground_truth_focal_length}}), full_x, full_y, no_gt));
        write_graph_as_svg(html, Graph("Second init frame", "Min Geometric Error", std::set<Curve>({{"Error", graph_min_error}, {"Error @gt", graph_error_at_ground_truth_focal_length}, {"Error @180fov", graph_error_for_max_focal_length}}), full_x, full_y, no_gt));
    }
    else {
       write_graph_as_svg(html, Graph("Second init frame", "Geometric Error Percent", std::set<Curve>({{"% error", graph_min_error_percent_max_focal_error}}), full_x, full_y, no_gt));
       write_graph_as_svg(html, Graph("Second init frame", "Min Geometric Error", std::set<Curve>({{"Error", graph_min_error}}), full_x, full_y, no_gt));
    }

    // Export the error against focal length and field of view for individual frame pairs
    if (true) {
        for (auto const& focal_error : p_focal_length_to_error.by_frame) {
            auto fov_error = p_fov_to_error.by_frame.find(focal_error.first);
            if (fov_error == p_fov_to_error.by_frame.end())
                continue;
            html << "<h3>Frame " << focal_error.first[0] << " and " << focal_error.first[1] << "</h3>\n";



            auto dec_focal_length_to_error = p_dec_focal_length_to_error.by_frame.find(focal_error.first);
            if (dec_focal_length_to_error == p_dec_focal_length_to_error.by_frame.end()) {

                write_graph_as_svg(html, Graph("Focal length (pixels)", "Geometric Error", std::set<Curve>({ {"Error", focal_error.second} }), focal_x, full_y, no_gt));
                write_graph_as_svg(html, Graph("FOV", "Geometric Error", std::set<Curve>({ {"Error", fov_error->second} }), full_x, full_y, no_gt));

            }
            else {
                //double scale(0.00001);
                //if (!dec_focal_length_to_error->second.empty() && !focal_error.second.empty())
                //    scale = focal_error.second.rbegin()->second / dec_focal_length_to_error->second.rbegin()->second;
                //auto rescaled_dec_error = rescale_graph(dec_focal_length_to_error->second, scale); // rescale to pixel error
                //range_behaviour range_y = full_y;
                //if (!rescaled_dec_error.empty())
                //    range_y = range_behaviour( 1.5 * std::max(rescaled_dec_error.begin()->second, rescaled_dec_error.rbegin()->second));

                double scale(100000);
                if (!dec_focal_length_to_error->second.empty() && !focal_error.second.empty())
                    scale = dec_focal_length_to_error->second.rbegin()->second / focal_error.second.rbegin()->second;
                auto rescaled_focal_error = rescale_graph(focal_error.second, scale); // rescale to pixel error
                axis_scaling range_y = full_y;
                if (!rescaled_focal_error.empty())
                    range_y = axis_scaling(1.5 * std::max(dec_focal_length_to_error->second.begin()->second, dec_focal_length_to_error->second.rbegin()->second));
                html << "<p>" << range_y.max << " " << dec_focal_length_to_error->second.begin()->second << " " << dec_focal_length_to_error->second.rbegin()->second << "</p>" << std::endl;
                write_graph_as_svg(html, Graph("Focal length (pixels)", "Geometric Error", std::set<Curve>({ {"Error", rescaled_focal_error}, {"Decomp Error", dec_focal_length_to_error->second} }), focal_x, range_y, no_gt));
            
                std::map<double, double> graph_dec_fov_to_error = focal_length_x_to_FOV_graph(dec_focal_length_to_error->second, video_width);
                auto rescaled_fov_error = rescale_graph(fov_error->second, scale); // rescale to pixel error

                write_graph_as_svg(html, Graph("FOV", "Geometric Error", std::set<Curve>({ {"Error", rescaled_fov_error}, {"Dec Error", graph_dec_fov_to_error} }), full_x, range_y, no_gt));

            
            }
        }
    }
}

void initialisation_debugging::save_html_report(std::string_view const& filename, std::optional<double> ground_truth_focal_length_x_pixels) const {
    html_file html(filename);
    add_to_html(html.html, ground_truth_focal_length_x_pixels);
}

std::optional<int> initialisation_debugging::average_init_frame_feature_count() const
{
    if (feature_count_by_frame.empty())
        return std::nullopt;
    int sum = std::accumulate(std::begin(feature_count_by_frame), std::end(feature_count_by_frame), 0,
                              [](int value, const std::map<int, int>::value_type& p){ return value + p.second; } );
    return int(0.5 + double(sum) / double(feature_count_by_frame.size()));
}

std::optional<int> initialisation_debugging::average_init_frame_unguided_match_count() const
{
    if (p_num_matches.by_frame.empty())
        return std::nullopt;
    double sum = std::accumulate(std::begin(p_num_matches.by_frame), std::end(p_num_matches.by_frame), 0.0,
        [](int value, const std::map<std::array<int, 2>, double>::value_type& p) { return value + p.second; });
    return int(0.5 + sum / double(p_num_matches.by_frame.size()));
}

/////////////////////////////////////////////////////////////////////////

thread_dubugging* thread_dubugging::get_instance() {
    if (!instance)
        instance = new thread_dubugging();
    return instance;
}

void thread_dubugging::set_thread_name(std::string name) {
    std::thread::id id = std::this_thread::get_id();
    auto f = thread_id_to_name.find(id);
    if (f == thread_id_to_name.end())
       thread_id_to_name[id] = name;
}

std::string thread_dubugging::thread_name() const {
    auto f = thread_id_to_name.find(std::this_thread::get_id());
    if (f != thread_id_to_name.end())
        return f->second;
    return "Unregistered";
}

} // namespace stella_vslam_bfx