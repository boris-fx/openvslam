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

std::set<double> candidateFocalLengthsOverFOVRange(double startDegrees, double endDegrees, double imageWidth)
{
   double const step(1);
   std::set<double> candidates;
   for (double fov = startDegrees; fov <= (endDegrees + 0.1); fov += step)
       candidates.insert(0.5 * imageWidth / tan(M_PI * fov / (2.0 * 180.0)));
   return candidates;
}

double error_for_focal_length(stella_vslam::Mat33_t const &F_21, camera::base const *camera, double focal_length_x_pixels)
{
    // Get the camera matrix for this focal length
    std::shared_ptr<stella_vslam::camera::base> modified_camera = modified_focal_length_camera_copy(camera, focal_length_x_pixels);
    Mat33_t const* cam_matrix = camera_eigen_cam_matrix(modified_camera.get());
    if (!cam_matrix)
        return -1.0;

    // Compute the error for this focal length
    double error = fundamental_focal_geometric_fit_error(F_21, *cam_matrix, *cam_matrix);

    return error;
}

template<typename LAMBDA_VALUE>
void RootFind_Bisection(float minX, float maxX, const LAMBDA_VALUE& lambdaValue) {

   const size_t c_numIterations = 25;

    float minY = lambdaValue(minX);
    float maxY = lambdaValue(maxX);

    if (minY > maxY) {
        std::swap(minX, maxX);
        std::swap(minY, maxY);
    }

    // y signs need to be opposite
    if (minY > 0.0f || maxY < 0.0f)
        return;

    for (size_t iterationIndex = 1; iterationIndex <= c_numIterations; ++iterationIndex) {
        float midX = (minX + maxX) / 2.0f;
        float midY = lambdaValue(midX);


        if (midY < 0.0f) {
            minX = midX;
            minY = midY;
        }
        else {
            maxX = midX;
            maxY = midY;
        }
    }
}


template<typename LAMBDA_VALUE>
void find_minimum_bisection(float minX, float maxX, const LAMBDA_VALUE& lambdaValue) {
    const size_t c_numIterations = 25;

    float minY = lambdaValue(minX);
    float maxY = lambdaValue(maxX);

    if (minY > maxY) {
        std::swap(minX, maxX);
        std::swap(minY, maxY);
    }

    // y signs need to be opposite
    if (minY > 0.0f || maxY < 0.0f)
        return;

    for (size_t iterationIndex = 1; iterationIndex <= c_numIterations; ++iterationIndex) {
        float midX = (minX + maxX) / 2.0f;
        float midY = lambdaValue(midX);

        if (midY < 0.0f) {
            minX = midX;
            minY = midY;
        }
        else {
            maxX = midX;
            maxY = midY;
        }
    }
}


bool initialize_focal_length(stella_vslam::Mat33_t const& F_21, camera::base* camera) {
   // Return early if auto focal length is turned off
   if (camera->autocalibration_parameters_.optimise_focal_length == false)
      return false;

   // Set of focal lengths (x-pixels)
   //std::set<double> candidates = { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1400, 1600, 2000, 3000, 4000, 5000 };
   std::set<double> candidates = candidateFocalLengthsOverFOVRange(1, 170, camera->cols_);

   std::map<double, double> focal_length_to_error; 
   std::map<double, double> fov_to_error; 
   for (auto const& candidate_focal_length : candidates) {

      double error = error_for_focal_length(F_21, camera, candidate_focal_length);

      focal_length_to_error[candidate_focal_length] = error;

      double fov = 2.0 * atan2(0.5 * (double)camera->cols_, candidate_focal_length) * 180.0 / M_PI;
      fov_to_error[fov] = error;
   }

   write_graphs_html("focal_length_error.html",
      {std::make_tuple("Focal length (pixels)", "Geometric Error", std::set<Curve>({{"Error", focal_length_to_error}})),
       std::make_tuple("FOV",                   "Geometric Error", std::set<Curve>({{"Error",          fov_to_error}})) });

   // Take the focal length value with smallest geometric error
   auto error_min = std::min_element(focal_length_to_error.begin(), focal_length_to_error.end(),
                                     [](const auto& a, const auto& b) { return a.second < b.second; });
   double focal_length_x_pixels_0 = error_min->first;

   spdlog::info("Initial focal length estimate: {}", focal_length_x_pixels_0);



   //auto Value = [](float t) -> float {return 3.4f;};
   //float minX, maxX;
   //RootFind_Bisection(minX, maxX, Value);


   auto before = error_min;
   --before;
   auto after = error_min;
   ++after;
   std::set<double> candidates_2;
   for (int i = (int)before->first - 1; i <= (int)after->first + 2; ++i)
       candidates_2.insert(i);

   std::map<double, double> focal_length_to_error_2;
   std::map<double, double> fov_to_error_2;
   for (auto const& candidate_focal_length : candidates_2) {
       double error = error_for_focal_length(F_21, camera, candidate_focal_length);

       focal_length_to_error_2[candidate_focal_length] = error;

       double fov = 2.0 * atan2(0.5 * (double)camera->cols_, candidate_focal_length) * 180.0 / M_PI;
       fov_to_error_2[fov] = error;
   }

   //write_graphs_html("focal_length_error_2.html",
   //                  {std::make_tuple("Focal length (pixels)", "Geometric Error", std::set<Curve>({{"Error", focal_length_to_error_2}})),
   //                   std::make_tuple("FOV", "Geometric Error", std::set<Curve>({{"Error", fov_to_error_2}}))});

   auto error_min_2 = std::min_element(focal_length_to_error_2.begin(), focal_length_to_error_2.end(),
                                     [](const auto& a, const auto& b) { return a.second < b.second; });
   double focal_length_x_pixels_2 = error_min_2->first;

   spdlog::info("Initial focal length estimate: {} -> {}", focal_length_x_pixels_0, focal_length_x_pixels_2);




   double delta_f(1);
   double error_plus = error_for_focal_length(F_21, camera, focal_length_x_pixels_2 + delta_f);
   double error_minus = error_for_focal_length(F_21, camera, focal_length_x_pixels_2 - delta_f);
   double de_df_plus = (error_plus - error_min->second) / delta_f;
   double de_df_minus = (error_min->second - error_minus) / delta_f;

   double error_for_max_focal_length = focal_length_to_error.rbegin()->second;
   double error_min_value = error_min->second;


   static int temp(0);
   ++temp; // frame
   static std::map<double, double> frame_to_error_for_max_focal_length; 
   static std::map<double, double> frame_to_min_error; 
   static std::map<double, double> frame_to_best_focalLength; 
   static std::map<double, double> frame_to_de_df_plus; 
   static std::map<double, double> frame_to_de_df_minus; 

   frame_to_error_for_max_focal_length[temp] = error_for_max_focal_length;
   frame_to_min_error[temp] = error_min_value;
   frame_to_best_focalLength[temp] = focal_length_x_pixels_2;
   frame_to_de_df_plus[temp] = -de_df_plus;
   frame_to_de_df_minus[temp] = de_df_minus;

   if (temp == 18) {
      write_graphs_html("baseline_focal_length_error.html",
                        {std::make_tuple("Baseline(frames)", "Error at tiny focal length", std::set<Curve>({{"Error", frame_to_error_for_max_focal_length}})),
                         std::make_tuple("Baseline(frames)", "Min Geometric Error", std::set<Curve>({{"Error", frame_to_min_error}})), 
                         std::make_tuple("Baseline(frames)", "Focal Confidence", std::set<Curve>({{"dE/dF(+)", frame_to_de_df_plus}, {"-dE/dF(-)", frame_to_de_df_minus}})),
                         std::make_tuple("Baseline(frames)", "Best Focal Length", std::set<Curve>({{"Focal length", frame_to_best_focalLength}})) });
   }

   bool set_f_ok = stella_vslam_bfx::setCameraFocalLength(camera, focal_length_x_pixels_2);
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
