#pragma once

#include <string>
#include <map>
#include <set>

using Curve = std::pair<std::string, std::map<double, double>>; /// Data name, map from x value to y
using Graph = std::tuple<std::string, std::string, std::set<Curve>>; /// x label, y label, set of curves

void write_graphs_html(std::string_view const& filename, std::set<Graph> graphs);



