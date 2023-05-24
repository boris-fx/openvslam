#include "fundamental_to_focal_length.h"

#include <fstream>

#include <spdlog/spdlog.h>

#include <stella_vslam/camera/base.h>
#include <stella_vslam/camera/fisheye.h>
#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/camera/radial_division.h>
#include <stella_vslam/data/map_camera_helpers.h>
#include <stella_vslam/report/plot_html.h>
#include <stella_vslam/report/metrics.h>

//

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#ifdef _WIN32
#define CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS // Avoids a compilation warning 
#endif

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"


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
            if (set_camera_focal_length_x_pixels(camera_copy.get(), focal_length_x_pixels))
               return camera_copy;
            break;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye const*>(camera);
            auto camera_copy = std::make_shared<camera::fisheye>(*c);
            if (set_camera_focal_length_x_pixels(camera_copy.get(), focal_length_x_pixels))
               return camera_copy;
            break;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division const*>(camera);
            auto camera_copy = std::make_shared<camera::radial_division>(*c);
            if (set_camera_focal_length_x_pixels(camera_copy.get(), focal_length_x_pixels))
               return camera_copy;
            break;
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

double error_for_focal_length(stella_vslam::Mat33_t const& F_21, double focal_length_x_pixels,
                              double par, double cx, double cy)
{
   double fx = focal_length_x_pixels;
   double fy = focal_length_x_pixels * par;
   Mat33_t cam_matrix;
   //cam_matrix << fx, 0, 0, 0, fy, 0, cx, cy, 1.0;
   cam_matrix << fx, 0, cx, 0, fy, cy, 0, 0, 1.0;

   // Compute the error for this focal length
   double error = fundamental_focal_geometric_fit_error(F_21, cam_matrix, cam_matrix);

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

    double error_for_max_focal_length = focal_length_to_error.rbegin()->second;
    double error_min_value = error_min->second;
    double min_error_percent_max_focal_error = 100.0 * error_min_value / error_for_max_focal_length;

    // If the calculation is stable there should be a large drop in error between a focal_length
    //    approaching infinity, and the focal length giving the minimum error
    double stability = error_for_max_focal_length / error_min_value;
    *focal_length_estimate_is_stable = stability > 2.0;
    //*focal_length_estimate_is_stable = error_for_max_focal_length > 0.04; // to be explored more

    spdlog::info("Initialisation focal length: error_for_max_focal_length {}, error_min_value {} (stability {})", error_for_max_focal_length, error_min_value, stability);

    spdlog::info("Initialisation focal length: nearest {}, bisection {} (stability {})", focal_length_x_pixels_0, focal_length_x_pixels_2, stability);

    if (metrics::initialisation_debug().active()) {

        double delta_f(1);
        double error_plus = error_for_focal_length(F_21, camera, focal_length_x_pixels_2 + delta_f);
        double error_minus = error_for_focal_length(F_21, camera, focal_length_x_pixels_2 - delta_f);
        double de_df_plus = -(error_plus - error_min->second) / delta_f;
        double de_df_minus = (error_min->second - error_minus) / delta_f;

        stella_vslam_bfx::metrics::initialisation_debug().video_width = (double)camera->cols_;
        metrics::initialisation_debug().submit_fundamental_to_focal_length_debugging(error_for_max_focal_length,
                                                                                     error_min_value,
                                                                                     focal_length_x_pixels_2,
                                                                                     focal_length_estimate_bisection,
                                                                                     de_df_plus,
                                                                                     de_df_minus,
                                                                                     min_error_percent_max_focal_error,
                                                                                     focal_length_to_error,
                                                                                     fov_to_error);

        //*focal_length_estimate_is_stable = false; // Make initialisation fail, so we can collect more initialisation data
        *focal_length_estimate_is_stable = true; // Try to make initialisation succeed, so we can collect parallax and fail a bit later to collect more initialisation data
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

   if (*focal_length_estimate_is_stable) {
       bool set_f_ok = stella_vslam_bfx::set_camera_focal_length_x_pixels(camera, focal_length_estimate);
       auto stage = stella_vslam_bfx::focal_estimation_type::initialisation_before_ba;
       stella_vslam_bfx::metrics::get_instance()->submit_intermediate_focal_estimate(stage, focal_length_estimate);
       return set_f_ok;
   }
   return false;
}



using namespace std;

G2O_USE_OPTIMIZATION_LIBRARY(dense);

double errorOfSolution(int numPoints, Eigen::Vector2d* points,
   const Eigen::Vector3d& circle) {
   Eigen::Vector2d center = circle.head<2>();
   double radius = circle(2);
   double error = 0.;
   for (int i = 0; i < numPoints; ++i) {
      double d = (points[i] - center).norm() - radius;
      error += d * d;
   }
   return error;
}

double errorOfSolution1D(int numPoints, Eigen::Vector2d* points,
   const Eigen::Matrix<double, 1, 1>& circle) {
   //   Eigen::Vector2d center = circle.head<2>();
   Eigen::Vector2d center(4.0, 2.0);
   double radius = circle(0);
   double error = 0.;
   for (int i = 0; i < numPoints; ++i) {
      double d = (points[i] - center).norm() - radius;
      error += d * d;
   }
   return error;
}
///**
// * \brief a circle located at x,y with radius r
// */
//class VertexCircle : public g2o::BaseVertex<3, Eigen::Vector3d> {
//public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//   VertexCircle() {}
//
//   virtual bool read(std::istream& /*is*/) { return false; }
//
//   virtual bool write(std::ostream& /*os*/) const { return false; }
//
//   virtual void setToOriginImpl() {
//      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
//   }
//
//   virtual void oplusImpl(const double* update) {
//      Eigen::Vector3d::ConstMapType v(update);
//      _estimate += v;
//   }
//};

namespace focal_error_minimisation {

   /**
    * \brief a circle located at x,y with radius r
    */
   class focal_length_vertex : public g2o::BaseVertex<1, Eigen::Matrix<double, 1, 1>> {
   public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      focal_length_vertex() {}

      virtual bool read(std::istream& /*is*/) { return false; }

      virtual bool write(std::ostream& /*os*/) const { return false; }

      virtual void setToOriginImpl() {
         cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      }

      virtual void oplusImpl(const double* update) {
         Eigen::Matrix<double, 1, 1>::ConstMapType v(update);
         _estimate += v;
      }
   };

   /**
    * \brief measurement for a point on the circle
    *
    * Here the measurement is the point which is on the circle.
    * The error function computes the distance of the point to
    * the center minus the radius of the circle.
    */
    //class EdgePointOnCircle
    //   : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle> {
    //public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //      EdgePointOnCircle() {}
    //   virtual bool read(std::istream& /*is*/) { return false; }
    //   virtual bool write(std::ostream& /*os*/) const { return false; }
    //
    //   template <typename T>
    //   bool operator()(const T* circle, T* error) const {
    //      typename g2o::VectorN<2, T>::ConstMapType center(circle);
    //      const T& radius = circle[2];
    //
    //      error[0] = (measurement().cast<T>() - center).norm() - radius;
    //      return true;
    //   }
    //
    //   G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
    //};



   class fundamental_matrix_edge
      : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, focal_length_vertex> {
   public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
         fundamental_matrix_edge() {}
      virtual bool read(std::istream& /*is*/) { return false; }
      virtual bool write(std::ostream& /*os*/) const { return false; }

      template <typename T>
      bool operator()(const T* focal_length, T* error) const {
         //typename g2o::VectorN<2, T>::ConstMapType center(circle);
         //Eigen::Matrix<T, 2, 1, 0>::ConstMapType centre(circle);
         Eigen::Vector2d center(4.0, 2.0);
         const T& radius = focal_length[0];

        // error[0] = error_for_focal_length(stella_vslam::Mat33_t const& F_21, camera::base const* camera, *focal_length);


         error[0] = (measurement().cast<T>() - center).norm() - radius;
         return true;
      }

      G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
   };

} // namespace focal_error_minimisation

bool fundamental_to_focal_length_optimisation_test()
{
   int numPoints = 100;
   int maxIterations = 10;
   bool verbose = false;

   // generate random data
   Eigen::Vector2d center(4.0, 2.0);
   double radius = 2.0;
   Eigen::Vector2d* points = new Eigen::Vector2d[numPoints];

   g2o::Sampler::seedRand();
   for (int i = 0; i < numPoints; ++i) {
      double r = g2o::Sampler::gaussRand(radius, 0.05);
      double angle = g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
      points[i].x() = center.x() + r * cos(angle);
      points[i].y() = center.y() + r * sin(angle);
   }

   // setup the solver
   g2o::SparseOptimizer optimizer;
   optimizer.setVerbose(false);

   // allocate the solver
   g2o::OptimizationAlgorithmProperty solverProperty;
   optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense",
         solverProperty));
   bool use_1D(true);
   focal_error_minimisation::focal_length_vertex* circle1D(nullptr);
   //VertexCircle* circle(nullptr);
   if (use_1D) {
      // build the optimization problem given the points
      // 1. add the circle vertex
      circle1D = new focal_error_minimisation::focal_length_vertex();
      circle1D->setId(0);
      circle1D->setEstimate(
         Eigen::Matrix<double, 1, 1>(3.0));  // some initial value for the circle
      optimizer.addVertex(circle1D);
      // 2. add the points we measured
      for (int i = 0; i < numPoints; ++i) {
         focal_error_minimisation::fundamental_matrix_edge* e = new focal_error_minimisation::fundamental_matrix_edge;
         e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
         e->setVertex(0, circle1D);
         e->setMeasurement(points[i]);
         optimizer.addEdge(e);
      }
   }
   else {
      //// build the optimization problem given the points
      //// 1. add the circle vertex
      //circle = new VertexCircle();
      //circle->setId(0);
      //circle->setEstimate(
      //   Eigen::Vector3d(3.0, 3.0, 3.0));  // some initial value for the circle
      //optimizer.addVertex(circle);
      //// 2. add the points we measured
      //for (int i = 0; i < numPoints; ++i) {
      //   EdgePointOnCircle* e = new EdgePointOnCircle;
      //   e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
      //   e->setVertex(0, circle);
      //   e->setMeasurement(points[i]);
      //   optimizer.addEdge(e);
      //}
   }

   // perform the optimization
   optimizer.initializeOptimization();
   optimizer.setVerbose(verbose);
   optimizer.optimize(maxIterations);

   if (verbose) cout << endl;

   // print out the result
   cout << "Iterative least squares solution" << endl;
   //if (circle) {
   //   cout << "center of the circle " << circle->estimate().head<2>().transpose()
   //      << endl;
   //   cout << "radius of the cirlce " << circle->estimate()(2) << endl;
   //   cout << "error " << errorOfSolution(numPoints, points, circle->estimate()) << endl;
   //}
   if (circle1D) {
      cout << "radius of the cirlce " << circle1D->estimate()(0) << endl;
      cout << "error " << errorOfSolution1D(numPoints, points, circle1D->estimate()) << endl;
   }

   cout << endl;

   // solve by linear least squares
   // Let (a, b) be the center of the circle and r the radius of the circle.
   // For a point (x, y) on the circle we have:
   // (x - a)^2 + (y - b)^2 = r^2
   // This leads to
   // (-2x -2y 1)^T * (a b c) = -x^2 - y^2   (1)
   // where c = a^2 + b^2 - r^2.
   // Since we have a bunch of points, we accumulate Eqn (1) in a matrix and
   // compute the normal equation to obtain a solution for (a b c).
   // Afterwards the radius r is recovered.
   Eigen::MatrixXd A(numPoints, 3);
   Eigen::VectorXd b(numPoints);
   for (int i = 0; i < numPoints; ++i) {
      A(i, 0) = -2 * points[i].x();
      A(i, 1) = -2 * points[i].y();
      A(i, 2) = 1;
      b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
   }
   Eigen::Vector3d solution =
      (A.transpose() * A).ldlt().solve(A.transpose() * b);
   // calculate the radius of the circle given the solution so far
   solution(2) = sqrt(pow(solution(0), 2) + pow(solution(1), 2) - solution(2));
   cout << "Linear least squares solution" << endl;
   cout << "center of the circle " << solution.head<2>().transpose() << endl;
   cout << "radius of the cirlce " << solution(2) << endl;
   cout << "error " << errorOfSolution(numPoints, points, solution) << endl;

   // clean up
   delete[] points;

   return true;
}


} // namespace stella_vslam_bfx
