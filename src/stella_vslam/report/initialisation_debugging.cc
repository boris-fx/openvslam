#include "initialisation_debugging.h"

#include <vector>
#include <cmath>

#include "plot_html.h"

namespace stella_vslam_bfx {

struct scalar_measurement {
    scalar_measurement(double value = -1.0, double variance = -1.0)
        : value(value), variance(variance) {}
    double value;
    double variance;
};

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

// NB: c++20 has std::lerp, which can replace this 
double naive_lerp(double a, double b, double t) {
    return a + t * (b - a);
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
{

}

bool initialisation_debugging::active() const {
    return is_active;
}

void initialisation_debugging::submit_feature_match_debugging(unsigned int num_matches)
{
    if (!is_active)
        return;

    timestamp_to_num_matches[current_init_frames] = double(num_matches);
}

void initialisation_debugging::submit_parallax_debugging(double parallax)
{
    if (!is_active)
        return;

    timestamp_to_parallax[current_init_frames] = parallax;
}

void initialisation_debugging::submit_homography_fundamental_cost(double cost_H, double cost_F)
{
    if (!is_active)
        return;

    timestamp_to_cost_H[current_init_frames] = cost_H;
    timestamp_to_cost_F[current_init_frames] = cost_F;
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

    timestamp_to_error_for_max_focal_length[current_init_frames] = error_for_max_focal_length;
    timestamp_to_min_error[current_init_frames] = min_error;
    timestamp_to_best_focal_length[current_init_frames] = best_focal_length;
    timestamp_to_best_focal_length_bisection[current_init_frames] = best_focal_length_bisection;
    timestamp_to_de_df_plus[current_init_frames] = de_df_plus;
    timestamp_to_de_df_minus[current_init_frames] = de_df_minus;
    timestamp_to_min_error_percent_max_focal_error[current_init_frames] = min_error_percent_max_focal_error;

    timestamp_to_focal_length_to_error[current_init_frames] = focal_length_to_error;
    timestamp_to_fov_to_error[current_init_frames] = fov_to_error;
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

void initialisation_debugging::create_frame_data(std::map<double, int> const& timestamp_to_video_frame) {

    transform_frame_data(timestamp_to_num_matches, frame_to_num_matches, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_parallax, frame_to_parallax, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_cost_H, frame_to_cost_H, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_cost_F, frame_to_cost_F, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_error_for_max_focal_length, frame_to_error_for_max_focal_length, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_min_error, frame_to_min_error, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_best_focal_length, frame_to_best_focal_length, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_best_focal_length_bisection, frame_to_best_focal_length_bisection, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_de_df_plus, frame_to_de_df_plus, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_de_df_minus, frame_to_de_df_minus, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_min_error_percent_max_focal_error, frame_to_min_error_percent_max_focal_error, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_focal_length_to_error, frame_to_focal_length_to_error, timestamp_to_video_frame);
    transform_frame_data(timestamp_to_fov_to_error, frame_to_fov_to_error, timestamp_to_video_frame);

    transform_frame_data(feature_count_by_timestamp, feature_count_by_frame, timestamp_to_video_frame);

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

std::map<double, double> constand_value_graph(std::map<double, double> const& data, double value) {
    std::map<double, double> output;
    if (!data.empty()) {
        output[data.begin()->first] = value;
        output[data.rbegin()->first] = value;
    }
    return output;
}

void initialisation_debugging::add_to_html(std::stringstream& html, std::optional<double> ground_truth_focal_length_x_pixels) const {
    if (!active())
        return;


    std::map<double, double> graph_num_matches = select_second_frame_data(frame_to_num_matches);
    std::map<double, double> graph_parallax = select_second_frame_data(frame_to_parallax);
    std::map<double, double> graph_cost_H = select_second_frame_data(frame_to_cost_H);
    std::map<double, double> graph_cost_F = select_second_frame_data(frame_to_cost_F);
    std::map<double, double> graph_error_for_max_focal_length = select_second_frame_data(frame_to_error_for_max_focal_length);
    std::map<double, double> graph_min_error = select_second_frame_data(frame_to_min_error);
    std::map<double, double> graph_de_df_plus = select_second_frame_data(frame_to_de_df_plus);
    std::map<double, double> graph_de_df_minus = select_second_frame_data(frame_to_de_df_minus);
    std::map<double, double> graph_min_error_percent_max_focal_error = select_second_frame_data(frame_to_min_error_percent_max_focal_error);
    std::map<double, double> graph_best_focal_length = select_second_frame_data(frame_to_best_focal_length);
    std::map<double, double> graph_best_focal_length_bisection = select_second_frame_data(frame_to_best_focal_length_bisection);

    std::map<double, double> graph_error_at_ground_truth_focal_length;
    std::map<double, double> graph_gt_error_percent_max_focal_error;
    std::map<double, double> graph_min_error_percent_gt_error;
    if (ground_truth_focal_length_x_pixels) {
        std::map<std::array<int, 2>, double> frame_to_error_at_ground_truth_focal_length
            = error_for_focal_length(ground_truth_focal_length_x_pixels.value(),
                                     frame_to_focal_length_to_error);
        graph_error_at_ground_truth_focal_length = select_second_frame_data(frame_to_error_at_ground_truth_focal_length);
        graph_gt_error_percent_max_focal_error = percent_of_value(graph_error_at_ground_truth_focal_length, graph_error_for_max_focal_length);
        graph_min_error_percent_gt_error = percent_deviation_from_value(graph_min_error, graph_error_at_ground_truth_focal_length);
    
    }

    std::map<double, double> graph_percent_matches;
    if (!frame_to_num_matches.empty()) {
        auto f = feature_count_by_frame.find(frame_to_num_matches.begin()->first[0]);
        if (f != feature_count_by_frame.end()) {
            double scale = 100.0 / double(f->second);
            graph_percent_matches = select_scaled_second_frame_data(frame_to_num_matches, scale);

        }
    }

    std::optional<double> percent_max(110);
    std::optional<double> focal_max(ground_truth_focal_length_x_pixels ? std::optional<double>(2.0 * ground_truth_focal_length_x_pixels.value()) : std::nullopt);
    std::optional<double> no_max;

    write_graph_as_svg(html, std::make_tuple("Second init frame", "Cost", std::set<Curve>({{"H cost", graph_cost_H}, {"F cost", graph_cost_F}}), no_max));
    write_graph_as_svg(html, std::make_tuple("Second init frame", "Num feature matches", std::set<Curve>({{"Match count", graph_num_matches}}), no_max));
    write_graph_as_svg(html, std::make_tuple("Second init frame", "Best Focal Length", std::set<Curve>({{"Focal length", graph_best_focal_length}, {"Focal length bisection", graph_best_focal_length_bisection}}), focal_max));
    if (ground_truth_focal_length_x_pixels) {
        std::map<double, double> graph_focal_deviation = percent_deviation_from_value(graph_best_focal_length_bisection, ground_truth_focal_length_x_pixels.value());
        std::set<Curve> curves({{"% Error in focal est.", graph_focal_deviation}, {"% Comp. error measure", graph_min_error_percent_max_focal_error}});
        if (!graph_percent_matches.empty())
            curves.insert({"Feature match %", graph_percent_matches});
        if (!graph_error_at_ground_truth_focal_length.empty()) {
            std::map<double, double> graph_focal_deviation_2 = percent_deviation_from_value(graph_error_at_ground_truth_focal_length, graph_min_error);
            curves.insert({"min error/gt error", graph_focal_deviation_2});
        }
        write_graph_as_svg(html, std::make_tuple("Second init frame", "Error Percent", curves, percent_max));
    }
    write_graph_as_svg(html, std::make_tuple("Second init frame", "Focal Confidence", std::set<Curve>({{"dE/dF(+)", graph_de_df_plus}, {"-dE/dF(-)", graph_de_df_minus}}), no_max));
    write_graph_as_svg(html, std::make_tuple("Second init frame", "Parallax", std::set<Curve>({{"Parallax", graph_parallax}}), no_max));
    write_graph_as_svg(html, std::make_tuple("Second init frame", "Error at tiny focal length", std::set<Curve>({{"Error", graph_error_for_max_focal_length}}), no_max));
    if (!graph_error_at_ground_truth_focal_length.empty()) {
        write_graph_as_svg(html, std::make_tuple("Second init frame", "Geometric Error Percent", std::set<Curve>({{"% error", graph_min_error_percent_max_focal_error}, {"% gt error", graph_gt_error_percent_max_focal_error}, {"min % gt error",  graph_min_error_percent_gt_error}}), no_max));
        write_graph_as_svg(html, std::make_tuple("Second init frame", "Min Geometric Error", std::set<Curve>({{"Error", graph_min_error}, {"Error @gt", graph_error_at_ground_truth_focal_length}}), no_max));
        write_graph_as_svg(html, std::make_tuple("Second init frame", "Min Geometric Error", std::set<Curve>({{"Error", graph_min_error}, {"Error @gt", graph_error_at_ground_truth_focal_length}, {"Error @180fov", graph_error_for_max_focal_length}}), no_max));
    }
    else {
       write_graph_as_svg(html, std::make_tuple("Second init frame", "Geometric Error Percent", std::set<Curve>({{"% error", graph_min_error_percent_max_focal_error}}), no_max));
       write_graph_as_svg(html, std::make_tuple("Second init frame", "Min Geometric Error", std::set<Curve>({{"Error", graph_min_error}}), no_max));
    }

    // Simple stochastic analysis
    {
        std::map<double, double> const& measurement(graph_best_focal_length_bisection);
        double uncertainty_multiplier(10.0 / 100.0);
        std::map<double, double> measurement_uncertainty = multiply(graph_min_error_percent_max_focal_error, measurement, uncertainty_multiplier);

        std::map<double, double> estimate, estimate_uncertainty;
        cumulative_stochastic_estimate(measurement, measurement_uncertainty, estimate, estimate_uncertainty);

        if (ground_truth_focal_length_x_pixels) {
            std::map<double, double> gt_graph = constand_value_graph(measurement, ground_truth_focal_length_x_pixels.value());
            write_graph_as_svg(html, std::make_tuple("Second init frame", "Stochastic estimates", std::set<Curve>({{"measurement", measurement}, {"estimate", estimate}, {"ground truth", gt_graph}}), no_max));
        }
        else
           write_graph_as_svg(html, std::make_tuple("Second init frame", "Stochastic estimates", std::set<Curve>({{"measurement", measurement} ,{"estimate", estimate}}), no_max));
        write_graph_as_svg(html, std::make_tuple("Second init frame", "Stochastic uncertainty", std::set<Curve>({{"measurement uncertainty", measurement_uncertainty}, {"estimate uncertainty", estimate_uncertainty}}), no_max));
    }

    // Export the error against focal length and field of view for individual frame pairs
    if (true) {
        for (auto const& focal_error : frame_to_focal_length_to_error) {
            auto fov_error = frame_to_fov_to_error.find(focal_error.first);
            if (fov_error == frame_to_fov_to_error.end())
                continue;
            html << "<h3>Frame " << focal_error.first[0] << " and " << focal_error.first[1] << "</h3>\n";
            write_graph_as_svg(html, std::make_tuple("Focal length (pixels)", "Geometric Error", std::set<Curve>({{"Error", focal_error.second}}), no_max));
            write_graph_as_svg(html, std::make_tuple("FOV", "Geometric Error", std::set<Curve>({{"Error", fov_error->second}}), no_max));
        }
    }
}

void initialisation_debugging::save_html_report(std::string_view const& filename, std::optional<double> ground_truth_focal_length_x_pixels) const {
    html_file html(filename);
    add_to_html(html.html, ground_truth_focal_length_x_pixels);
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