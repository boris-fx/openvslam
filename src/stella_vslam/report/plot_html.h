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

using Curve = std::pair<std::string, std::map<double, double>>; /// Data name, map from x value to y
using Graph = std::tuple<std::string, std::string, std::set<Curve>, std::optional<double>>; /// x label, y label, set of curves, hard max Y

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
