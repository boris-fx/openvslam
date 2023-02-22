#pragma once

#include <string>
#include <map>
#include <set>
#include <thread>

using Curve = std::pair<std::string, std::map<double, double>>; /// Data name, map from x value to y
using Graph = std::tuple<std::string, std::string, std::set<Curve>>; /// x label, y label, set of curves

void write_graphs_html(std::string_view const& filename, std::set<Graph> graphs);

bool disable_all_html_graph_export(); /// Temporary - not sure where to control this

namespace stella_vslam_bfx {

class metrics_and_debugging {
private:
    inline static metrics_and_debugging* instance{nullptr};
    metrics_and_debugging() = default;
    ~metrics_and_debugging() = default;

    std::map<std::thread::id, std::string> thread_id_to_name; // protect with mutex

public:

    std::string thread_name() const;
    void set_thread_name(std::string name);

    metrics_and_debugging(const metrics_and_debugging&) = delete;
    metrics_and_debugging& operator=(const metrics_and_debugging&) = delete;

    static metrics_and_debugging* get_instance();
};

} // namespace stella_vslam_bfx