#include "metrics.h"

#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <nlohmann/json.hpp>

#include "plot_html.h"

#include "stella_vslam/solve/fundamental_consistency.h"

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

    //////////////////////////////////////////////////////////////

    const double metrics::par_percent_error_trigger = 20.0;
    const double metrics::focal_percent_error_trigger = 50.0;

    metrics* metrics::get_instance() {
        if (!instance)
            instance = new metrics();
        return instance;
    }

    void metrics::clear() {
        if (instance) {
            delete instance;
            instance = nullptr;
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

    std::optional<stage_and_frame> timestamp_to_frame(double timestamp, std::map<double, stage_and_frame> const& timestamp_to_stage_and_frame)
    {
        auto f = timestamp_to_stage_and_frame.find(timestamp);
        if (f != timestamp_to_stage_and_frame.end())
            return f->second;
        return std::nullopt;
    }

    void metrics::submit_intermediate_focal_estimate(focal_estimation_type type, double estimate)
    {
        if (!timestamp_to_stage_and_frame)
            return;
        std::optional<stage_and_frame> stage_with_frame = timestamp_to_frame(current_frame_timestamp, *timestamp_to_stage_and_frame);
        if (!stage_with_frame)
            return;

        intermediate_focal_estimates.push_back({ estimate, type, stage_with_frame.value() });
    }

    void metrics::submit_map_size_and_tracking_fails(double timestamp, unsigned int map_keyframe_count, unsigned int tracking_fails)
    {
        if (!timestamp_to_stage_and_frame)
            return;
        std::optional<stage_and_frame> stage_with_frame = timestamp_to_frame(timestamp, *timestamp_to_stage_and_frame);
        if (!stage_with_frame)
            return;
        
        map_size.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = map_keyframe_count;
        tracking_fail_count.by_stage_and_frame[stage_with_frame.value().stage][stage_with_frame.value().frame] = tracking_fails;

        //map_size.by_timestamp[timestamp] = map_keyframe_count;
        //tracking_fail_count.by_timestamp[timestamp] = tracking_fails;
    }

    void metrics::submit_mapping_reset(double timestamp)
    {
        mapping_reset_timestamps.push_back(timestamp);
    }

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
                auto f = timestamp_to_stage_and_frame->find(timestamp);
                if (f != timestamp_to_stage_and_frame->end())
                    initialisation_attempt_frames.insert(f->second.frame);
            }
            initialisation_frames.insert(initialisation_attempt_frames);
        }

        initialisation_debug_object.create_frame_data(*timestamp_to_stage_and_frame);

        //for (auto& estimate : intermediate_focal_estimates)
        //    if (auto frame = timestamp_to_frame(estimate.timestamp, *timestamp_to_stage_and_frame))
        //        estimate.frame = frame.value().frame;

        mapping_reset_frames.clear();
        for (auto const& timestamp : mapping_reset_timestamps)
            if (auto frame = timestamp_to_frame(timestamp, *timestamp_to_stage_and_frame))
                mapping_reset_frames.push_back(frame.value().frame);

        //transform_metrics_frame_data(map_size, *timestamp_to_stage_and_frame);
        //transform_metrics_frame_data(tracking_fail_count, *timestamp_to_stage_and_frame);
    }

    int metrics::total_frames() const
    {
        return solved_frame_count + unsolved_frame_count;
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

void metrics::save_html_report(std::string_view const& filename, std::string thumbnail_path_relative, std::string video_path_relative,
    std::optional<double> known_focal_length_x_pixels) const {
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

    if (debugging.debug_initialisation) {
        html << "<h2>Initialisation debug</h2>\n";
        initialisation_debug_object.add_to_html(html, input_video_metadata.ground_truth_focal_length_x_pixels());
    }

    // Number of features and matches by frame
    std::map<double, double> graph_num_matches = select_second_frame_data(initialisation_debug_object.p_num_matches.by_frame);
    std::map<double, double> graph_feature_count;
    for (auto const& i : initialisation_debug_object.feature_count_by_frame)
        graph_feature_count[i.first] = i.second;
    write_graph_as_svg(html, Graph("Second init frame", "Num feature matches", std::set<Curve>({ {"Feature count", graph_feature_count}, {"Matches to frame", graph_num_matches} }), range_behaviour::no_max, range_behaviour::no_max, settings.min_num_valid_pts_));
    html << "<hr>" << std::endl;

    // Structure type (planar or non-planar) costs during two-view matching 
    std::map<double, double> graph_cost_H = select_second_frame_data(initialisation_debug_object.p_cost_H.by_frame);
    std::map<double, double> graph_cost_F = select_second_frame_data(initialisation_debug_object.p_cost_F.by_frame);
    html << "<h2>Structure type</h2>" << std::endl;
    write_graph_as_svg(html, Graph("Second init frame", "Cost", std::set<Curve>({ {"Planar error", graph_cost_H}, {"Non-planar error", graph_cost_F} }), range_behaviour::no_max, range_behaviour::no_max, std::nullopt));
    html << "<p>Average feature match deviation from a planar or non-planar geometric model (pixels - with max of a few pixels).</p>" << std::endl;
    html << "<hr>" << std::endl;

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

    // Intermediate focal length estimates
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

    html << "<h2> Map</h2>\n";
    // Solved/unsolved cameras
    html << "<p> " << solved_frame_count << " cameras created from the " << total_frames() << " frames of video";
    if (unsolved_frame_count!=0)
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

    // Timing
    html << "<h2> Timing</h2>\n";
    std::optional<double> fps(total_frames() == 0 ? std::nullopt : std::optional<double>((double(total_frames()) / track_timings.total_time_sec())));
    html << "<p> Total tracking time:" << track_timings.total_time_sec() << " sec";
    if (fps)
        html << " (" << fps.value() << "fps)";
    html << "</p>\n ";
    html << "<p> Forward mapping: " << track_timings.forward_mapping << ".</p>\n";
    html << "<p> Backward mapping: " << track_timings.backward_mapping << ".</p>\n";
    html << "<p> Loop Closing: " << track_timings.loop_closing << ".</p>\n";
    html << "<p> Optimisation: " << track_timings.optimisation << ".</p>\n";
    html << "<p> Final Tracking: " << track_timings.tracking << ".</p>\n";

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
        bool fail(m->solved_frame_count == 0);
        std::optional<double> fps(m->total_frames() == 0 ? std::nullopt : std::optional<double>((double(m->total_frames()) / m->track_timings.total_time_sec())));
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
        html << "      <text>" << m->total_frames() << " frames, " << m->track_timings.total_time_sec() << " sec";
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
