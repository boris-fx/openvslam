/** \file
 * \brief Camera tracking metrics recorded during and ater tracking
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <string>
#include <map>
#include <set>
#include <sstream>
#include <fstream>
#include <optional>

enum class range_behaviour { no_max, hard_max, max_from_median };
struct axis_scaling
{
    axis_scaling(range_behaviour behaviour) : behaviour(behaviour), max(0) {}
    axis_scaling(double max) : behaviour(range_behaviour::hard_max), max(max) {}
    range_behaviour behaviour;
    double          max;
};

using Curve = std::pair<std::string, std::map<double, double>>; /// Data name, map from x value to y
//using Graph = std::tuple<std::string, std::string, std::set<Curve>, axis_scaling, std::optional<double>>; /// x label, y label, set of curves, Y-axis scaling, ground truth y-value

struct Graph
{
    Graph(std::string x_label, std::string y_label, std::set<Curve> curves, axis_scaling x_axis_scaling, axis_scaling y_axis_scaling, std::optional<double> ground_truth_y=std::nullopt)
        : x_label(x_label), y_label(y_label), curves(curves), ground_truth_y(ground_truth_y)
        , x_axis_scaling(x_axis_scaling), y_axis_scaling(y_axis_scaling)
    {}

    std::string x_label;
    std::string y_label;
    std::set<Curve> curves;
    axis_scaling x_axis_scaling;
    axis_scaling y_axis_scaling;
    std::optional<double> ground_truth_y;


};

void write_graphs_html(std::string_view const& filename, std::set<Graph> graphs);

void write_graph_as_svg(std::stringstream& svg, Graph const& graph);

bool disable_all_html_graph_export(); /// Temporary - not sure where to control this

class html_file {
public:
    html_file(std::string_view const& filename);
    ~html_file();
    template<typename T> void operator<<(T const& data) { html << data; }
    std::stringstream html;
protected:
    std::ofstream myfile;
};
