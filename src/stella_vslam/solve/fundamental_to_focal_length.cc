#include "fundamental_to_focal_length.h"

#include <fstream>

#include <spdlog/spdlog.h>

#include <stella_vslam/camera/base.h>
#include <stella_vslam/camera/fisheye.h>
#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/camera/radial_division.h>
#include <stella_vslam/data/keyframe_autocalibration_wrapper.h>
#include <stella_vslam/util/plot_html.h>

using namespace stella_vslam;

namespace stella_vslam_bfx {

//! Given a fundamental matrix and camera intrinsics return a measure of deviation from geometric consistency (second singular value of the essential matrix)
double fundamental_focal_geometric_fit_error(const Mat33_t& F_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2)
{
   const Mat33_t E_21 = cam_matrix_2.transpose() * F_21 * cam_matrix_1;

   const Eigen::JacobiSVD<Mat33_t> svd(E_21, Eigen::ComputeFullU | Eigen::ComputeFullV);

   Vec3_t s = svd.singularValues();

   double error =  (s[0] - s[1]) / s[1]; // slightly arbitrary

   return error;
}

// NB: this is in the initialize::perspective class, but is private
Mat33_t get_camera_matrix(camera::base* camera) {
    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera);
            return c->eigen_cam_matrix_;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            return c->eigen_cam_matrix_;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division*>(camera);
            return c->eigen_cam_matrix_;
        }
        default: {
            throw std::runtime_error("Cannot get a camera matrix from the camera model");
        }
    }
}

std::shared_ptr<stella_vslam::camera::base> modified_focal_length_camera_copy(stella_vslam::camera::base const* camera, double focal_length_x_pixels)
{
   switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective const*>(camera);
            auto camera_copy = std::make_shared<camera::perspective>(*c);
            if (setCameraFocalLength(camera_copy.get(), focal_length_x_pixels))
               return camera_copy;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye const*>(camera);
            auto camera_copy = std::make_shared<camera::fisheye>(*c);
            if (setCameraFocalLength(camera_copy.get(), focal_length_x_pixels))
               return camera_copy;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division const*>(camera);
            auto camera_copy = std::make_shared<camera::radial_division>(*c);
            if (setCameraFocalLength(camera_copy.get(), focal_length_x_pixels))
               return camera_copy;
        }
    }
    return nullptr;
}

Mat33_t const* camera_eigen_cam_matrix(stella_vslam::camera::base const* camera)
{
    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective const*>(camera);
            return &c->eigen_cam_matrix_;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye const*>(camera);
            return &c->eigen_cam_matrix_;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division const*>(camera);
            return &c->eigen_cam_matrix_;
        }
    }
   return nullptr;
}

void write_graph(std::string_view const& filename,
   std::map<double, double> const& focal_length_data,
   std::map<double, double> const& fov_data) {
   std::ofstream myfile;
   myfile.open(filename.data());

   std::stringstream html;
   html << "<!DOCTYPE html><html><head></head><body>";

   {
       std::list<LabelledCurve> curves;

       LabelledCurve errorCurve;
       errorCurve.name = "Geometric Error";
       errorCurve.dataValues.resize(focal_length_data.size());
       int i(0);
       for (auto const& vertex : focal_length_data) {
           errorCurve.dataValues[i].index = int(vertex.first + 0.5);
           errorCurve.dataValues[i].value = (float)vertex.second;
           //errorCurve.dataValues[i].label = std::string();
           ++i;
       }
       curves.push_back(errorCurve);

       bool linearXLabels(true);
       addLabelToValueGraphToHtml(html, "Geometric Error", "", "Focal length (pixels)", curves, linearXLabels, 1200, 600, true);
   }

   {
       std::list<LabelledCurve> curves;

       LabelledCurve errorCurve;
       errorCurve.name = "Geometric Error";
       errorCurve.dataValues.resize(fov_data.size());
       int i(0);
       for (auto const& vertex : fov_data) {
           errorCurve.dataValues[i].index = int(vertex.first + 0.5);
           errorCurve.dataValues[i].value = (float)vertex.second;
           //errorCurve.dataValues[i].label = std::string();
           ++i;
       }
       curves.push_back(errorCurve);

       bool linearXLabels(true);
       addLabelToValueGraphToHtml(html, "Geometric Error", "", "Field of View", curves, linearXLabels, 1200, 600, true);
   }

   html << "</body></html>";

   myfile << html.str();
   myfile.close();
}

std::set<double> candidateFocalLengthsOverFOVRange(double startDegrees, double endDegrees, double imageWidth)
{
   double const step(1);
   std::set<double> candidates;
   for (double fov = startDegrees; fov <= (endDegrees + 0.1); fov += step)
       candidates.insert(0.5 * imageWidth / tan(M_PI * fov / (2.0 * 180.0)));
   return candidates;
}

bool initialize_focal_length(stella_vslam::Mat33_t const& F_21, camera::base* camera) {
   // Return early if auto focal length is turned off
   if (camera->autocalibration_parameters_.optimise_focal_length == false)
      return false;

   // Set of focal lengths (x-pixels)
   //std::set<double> candidates = { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1400, 1600, 2000, 3000, 4000, 5000 };
   std::set<double> candidates = candidateFocalLengthsOverFOVRange(4, 170, camera->cols_);

   std::map<double, double> focal_length_to_error; 
   std::map<double, double> fov_to_error; 
   for (auto const& candidate_focal_length : candidates) {

      // Get the camera matrix for this focal length
      std::shared_ptr<stella_vslam::camera::base> candidate_camera = modified_focal_length_camera_copy(camera, candidate_focal_length);
      Mat33_t const* cam_matrix = camera_eigen_cam_matrix(candidate_camera.get());
      if (!cam_matrix)
          continue;
      
      // Compute the error for this focal length
      double error = fundamental_focal_geometric_fit_error(F_21, *cam_matrix, *cam_matrix);

      focal_length_to_error[candidate_focal_length] = error;


      double fov = 2.0 * atan2(0.5 * (double)camera->cols_, candidate_focal_length) * 180.0 / M_PI;
      fov_to_error[fov] = error;
   }

   write_graph("focal_length_error.html", focal_length_to_error, fov_to_error);

   // Take the focal length value with smallest geometric error
   auto error_min = std::min_element(focal_length_to_error.begin(), focal_length_to_error.end(),
                                     [](const auto& a, const auto& b) { return a.second < b.second; });
   double focal_length_x_pixels = error_min->first;

   spdlog::info("Initial focal length estimate: {}", focal_length_x_pixels);

   bool set_f_ok = stella_vslam_bfx::setCameraFocalLength(camera, focal_length_x_pixels);
   return set_f_ok;
}



} // namespace stella_vslam_bfx





#if 0
#include <stella_vslam/util/svg_plot/svg_2d_plot.hpp>
// boost::svg::svg_2d_plot
#include <limits>
// infinity
#include <map>
//using std::map;

double f(double x) { // Function to plot.
    return 1. / x;
}

int plot_test() {
    using namespace boost::svg; // For SVG named colors.
    std::map<double, double> data1;

    const double interval = 0.5;
    for (double i = -10; i <= 10.; i += interval) {
        data1[i] = f(i);
    }

    svg_2d_plot my_plot;

    // Image size & ranges settings.
    my_plot.size(500, 350)    // SVG image in pixels.
        .x_range(-10.5, 10.5) // Offset by 0.5 so that +10 and -10 markers are visible.
        .y_range(-1.1, 1.1);  // Offset by 1 so that +10 and -10 markers are visible.

    // Text settings.
    my_plot.title("Plot of 1 / x")
        .x_label("X Axis Units")
        .y_label("F(x)")
        .y_major_labels_side(-1) // Left.
        .plot_window_on(true);

    // X-axis settings.
    my_plot.x_major_interval(2)
        .x_major_tick_length(14)
        .x_major_tick_width(1)
        .x_minor_tick_length(7)
        .x_minor_tick_width(1)
        .x_num_minor_ticks(1)
        .x_major_labels_side(1) // Top of X-axis line (but zero collides with vertical x == 0 line).

        // Y-axis settings.
        .y_major_interval(1)
        .y_num_minor_ticks(4);

    // Legend-box settings.
    my_plot.legend_title_font_size(15);

    // Limit value at x = 0 when 1/x == +infinity shown by a pointing-down cone a the top of the plot.
    //my_plot.minus_inf_limit_color(red).plus_inf_limit_color(green);
    // TODO - the 'at-limit' infinity point cone pointing-down shows as default color pink, not the changed color(s).

    my_plot.plot(data1, "1 / x").shape(square).size(5).line_on(false);

    my_plot.write("./2d_limit.svg");

    // Sets and gets colors correctly, but not used? A fix would be good, but does display.
    // std::cout << "" << my_plot.plus_inf_limit_color() << std::endl; // RGB(0,128,0)
    //std::cout << "" << my_plot.minus_inf_limit_color() << std::endl; // RGB(255,0,0)

    return 0;
} // int main()



#endif
