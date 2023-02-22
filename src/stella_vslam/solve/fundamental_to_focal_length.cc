#include "fundamental_to_focal_length.h"

#include <fstream>

#include <spdlog/spdlog.h>

#include <stella_vslam/camera/base.h>
#include <stella_vslam/camera/fisheye.h>
#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/camera/radial_division.h>
#include <stella_vslam/data/keyframe_autocalibration_wrapper.h>
#include <stella_vslam/report/plot_html.h>

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

// only works when close to the minimum
double min_geometric_error_focal_length_bisection(stella_vslam::Mat33_t const& F_21, camera::base const* camera, double focal_x_start, double focal_x_end)
{
   int const max_iterations = 20;

   //double start_fov_degrees(1), end_fov_degrees(170);
   double f_0 = focal_x_start;  ////0.5 * double(camera->cols_) / tan(M_PI * end_fov_degrees   / (2.0 * 180.0));
   double e_0 = error_for_focal_length(F_21, camera, f_0);
   double f_1 = focal_x_end;  ////0.5 * double(camera->cols_) / tan(M_PI * start_fov_degrees / (2.0 * 180.0));
   double e_1 = error_for_focal_length(F_21, camera, f_1);

   for (int i = 0; i < max_iterations; ++i) {

      double f_m = 0.5 * (f_0 + f_1);
      double e_m = error_for_focal_length(F_21, camera, f_m);

      if (e_0 < e_1) {
          if (e_1 < e_m)
              break;
          f_1 = f_m;
          e_1 = e_m;
      }
      else {
          if (e_0 < e_m)
              break;
          f_0 = f_m;
          e_0 = e_m;
      }

   }

   return 0.5 * (f_0 + f_1);
}

template<typename Iter, typename Cont>
bool is_last(Iter iter, const Cont& cont) {
    if (iter == cont.end())
        return false;
    Iter next = iter;
    ++next;
    return next == cont.end();
}

double min_geometric_error_focal_length(stella_vslam::Mat33_t const& F_21, camera::base const* camera, bool* focal_length_estimate_is_stable) {
    // Set of focal lengths (x-pixels)
    std::set<double> candidates = candidateFocalLengthsOverFOVRange(1, 170, camera->cols_);

    std::map<double, double> focal_length_to_error;
    std::map<double, double> fov_to_error;
    for (auto const& candidate_focal_length : candidates) {
        double error = error_for_focal_length(F_21, camera, candidate_focal_length);

        focal_length_to_error[candidate_focal_length] = error;

        double fov = 2.0 * atan2(0.5 * (double)camera->cols_, candidate_focal_length) * 180.0 / M_PI;
        fov_to_error[fov] = error;
    }


    // Take the focal length value with smallest geometric error
    auto error_min = std::min_element(focal_length_to_error.begin(), focal_length_to_error.end(),
                                      [](const auto& a, const auto& b) { return a.second < b.second; });
    double focal_length_x_pixels_0 = error_min->first;

    spdlog::info("Initial focal length estimate: {}", focal_length_x_pixels_0);

    auto before = error_min;
    if (before != focal_length_to_error.begin())
       --before;
    auto after = error_min;
    if (!is_last(after, focal_length_to_error))
       ++after;
    std::set<double> candidates_2;
    for (int i = (int)before->first - 1; i <= (int)after->first + 2; ++i)
        candidates_2.insert(i);

    double focal_length_estimate_bisection = min_geometric_error_focal_length_bisection(F_21, camera, *candidates_2.begin(), *candidates_2.rbegin()); // only works when close to the minimum

    std::map<double, double> focal_length_to_error_2;
    std::map<double, double> fov_to_error_2;
    for (auto const& candidate_focal_length : candidates_2) {
        double error = error_for_focal_length(F_21, camera, candidate_focal_length);

        focal_length_to_error_2[candidate_focal_length] = error;

        double fov = 2.0 * atan2(0.5 * (double)camera->cols_, candidate_focal_length) * 180.0 / M_PI;
        fov_to_error_2[fov] = error;
    }

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

    *focal_length_estimate_is_stable = error_for_max_focal_length > 0.04; // to be explored more

    static int frame_hit(0);
    ++frame_hit; // frame_hit=7 for matching between frames 0 and 7
    static std::map<double, double> frame_to_error_for_max_focal_length;
    static std::map<double, double> frame_to_min_error;
    static std::map<double, double> frame_to_best_focal_length;
    static std::map<double, double> frame_to_best_focal_length_bisection;
    static std::map<double, double> frame_to_de_df_plus;
    static std::map<double, double> frame_to_de_df_minus;

    frame_to_error_for_max_focal_length[frame_hit] = error_for_max_focal_length;
    frame_to_min_error[frame_hit] = error_min_value;
    frame_to_best_focal_length[frame_hit] = focal_length_x_pixels_2;
    frame_to_best_focal_length_bisection[frame_hit] = focal_length_estimate_bisection;
    frame_to_de_df_plus[frame_hit] = -de_df_plus;
    frame_to_de_df_minus[frame_hit] = de_df_minus;

    if (frame_hit == 18 && !disable_all_html_graph_export()) {
        write_graphs_html("baseline_focal_length_error.html",
                          {std::make_tuple("Baseline(frames)", "Error at tiny focal length", std::set<Curve>({{"Error", frame_to_error_for_max_focal_length}})),
                           std::make_tuple("Baseline(frames)", "Min Geometric Error", std::set<Curve>({{"Error", frame_to_min_error}})),
                           std::make_tuple("Baseline(frames)", "Focal Confidence", std::set<Curve>({{"dE/dF(+)", frame_to_de_df_plus}, {"-dE/dF(-)", frame_to_de_df_minus}})),
                           std::make_tuple("Baseline(frames)", "Best Focal Length", std::set<Curve>({{"Focal length", frame_to_best_focal_length}, {"Focal length bisection", frame_to_best_focal_length_bisection}}))});
    }

    if (!disable_all_html_graph_export()) {
       write_graphs_html("focal_length_error_0_" + std::to_string(frame_hit) + "_" + std::to_string((int)(focal_length_x_pixels_2 + 0.5)) + ".html",
                         {std::make_tuple("Focal length (pixels)", "Geometric Error", std::set<Curve>({{"Error", focal_length_to_error}})),
                          std::make_tuple("FOV", "Geometric Error", std::set<Curve>({{"Error", fov_to_error}}))});
    }
    return focal_length_x_pixels_2;
}

bool initialize_focal_length(stella_vslam::Mat33_t const& F_21, camera::base* camera, bool* focal_length_estimate_is_stable) {
   // Return early if auto focal length is turned off
   if (camera->autocalibration_parameters_.optimise_focal_length == false) {
      *focal_length_estimate_is_stable = true;
      return false;
   }
   double focal_length_estimate = min_geometric_error_focal_length(F_21, camera, focal_length_estimate_is_stable);

   if (focal_length_estimate_is_stable) {
       bool set_f_ok = stella_vslam_bfx::setCameraFocalLength(camera, focal_length_estimate);
       return set_f_ok;
   }
   return false;
}

} // namespace stella_vslam_bfx
