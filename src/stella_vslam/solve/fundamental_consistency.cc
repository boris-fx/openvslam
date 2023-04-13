#include "fundamental_consistency.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/auto_differentiation.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>

#undef G2O_LINEAR_SOLVER_CLASS
#if defined(HAVE_G2O_SOLVER_CSPARSE)
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#define G2O_LINEAR_SOLVER_CLASS LinearSolverCSparse 
#else
#define G2O_LINEAR_SOLVER_CLASS LinearSolverEigen
#endif

#include <g2o/core/optimization_algorithm_levenberg.h>

//#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

#include <stella_vslam/camera/perspective.h>
#include <stella_vslam/data/keyframe_autocalibration_wrapper.h>
#include "stella_vslam/solve/essential_solver.h"
#include "stella_vslam/util/converter.h"
#include "stella_vslam/report/metrics.h"
#include "stella_vslam/optimize/internal/camera_intrinsics_vertex.h"
#include "stella_vslam/optimize/internal/se3/shot_vertex.h"

#include "fundamental_to_focal_length.h"

namespace g2o {
    namespace bal {
        using Vector6 = VectorN<6>;
    }
}  // namespace g2o


#define USE_TRANSLATION_DIRECTION 1

namespace stella_vslam_bfx {

void run_epipolar_optimisation(input_matches const& matches, stella_vslam::camera::base const* camera,
                                unsigned int num_iter, bool* const force_stop_flag);

// == Notes on the orthonormal representation of F ==
//
// - Take the SVD of F to give USV' (U * S * transpose of V)
// - S is a diagonal matrix containing the eigenvalues of F
// - Since F is rank 2, S = diag(sigma1, sigma2, 0)
// - We can also set the (arbitrary) scale such that S = diag(1, sigma, 0), where sigma = sigma2/sigma1
// - Sigma should be between 0 and 1 (A sigma of 1 means the matrix is an essential matrix, and the orthonormal representation is not unique)
// - The orthonormal representation is then (U, V, sigma)
// - Updates are parameterised by 7 scalar values (x1, x2, x3, y1, y2, y3, delta_sigma)
//    (x1, x2, x3) and (y1, y2, y3) are two sets of euler angles used to generate 3x3 rotation matrices Rx and Ry
//    Updates are applied to F as follows:
//     U <- U * Rx
//     V <- V * Ry
//     sigma <- sigma + delta_sigma
// 
void orthonormal_f::from_fundamental_matrix(const Eigen::Matrix3d& F)
{
   const Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
   u = svd.matrixU();
   v = svd.matrixV();
   Eigen::Vector3d s = svd.singularValues();
   sigma = s(1) / s(0);
}

void orthonormal_f::to_fundamental_matrix(Eigen::Matrix3d& f) const
{
    Eigen::Vector3d s;
    s << 1, sigma, 0;
    f = u * s.asDiagonal() * v.transpose();
}

Eigen::Matrix3d eulerZYX_2_rot(double roll_rad, double pitch_rad, double yaw_rad)
{
    // ZYX order, same as rot_z(yaw_rad) * rot_y(pitch_rad) * rot_x(roll_rad)
    double sx = sin(roll_rad);
    double cx = cos(roll_rad);
    double sy = sin(pitch_rad);
    double cy = cos(pitch_rad);
    double sz = sin(yaw_rad);
    double cz = cos(yaw_rad);
    Eigen::Matrix3d rot(3, 3);
    rot(0, 0) = cz * cy;
    rot(0, 1) = cz * sy * sx - sz * cx;
    rot(0, 2) = cz * sy * cx + sz * sx;
    rot(1, 0) = sz * cy;
    rot(1, 1) = sz * sy * sx + cz * cx;
    rot(1, 2) = sz * sy * cx - cz * sx;
    rot(2, 0) = -sy;
    rot(2, 1) = cy * sx;
    rot(2, 2) = cy * cx;
    return rot;
}

void orthonormal_f::update(Eigen::Matrix<double, 7, 1> const& delta)
{
    u = u * eulerZYX_2_rot(delta(0), delta(1), delta(2)); // u <- u * rotation(delta(0), delta(1), delta(2))
    v = v * eulerZYX_2_rot(delta(3), delta(4), delta(5)); // v <- v * rotation(delta(3), delta(4), delta(5))
    sigma += delta(6); // sigma <- sigma + delta(6)
}

bool orthonormal_f_test(Eigen::Matrix3d const& f)
{
    orthonormal_f ortho_f;
    ortho_f.from_fundamental_matrix(f);

    Eigen::Matrix3d f_check;
    ortho_f.to_fundamental_matrix(f_check);




    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

const double PI = 3.1415926535897932384626433832795028841971693993751058209749;

template<typename T>
T getPhi(T y, T x)
{
    if (x == 0.0) {
        if (y == 0.0) {
            return 0.0;
        }
        else if (y > 0.0) {
            return T(PI) / 2.0;
        }
        else {
            return -1.0 * T(PI) / 2.0;
        }
    }
    else if (x > 0.0) {
        return atan(y / x);
    }
    else if (x < 0.0) {
        if (y >= 0.0) {
            return atan(y / x) + T(PI);
        }
        else {
            return atan(y / x) + T(PI);
        }
    }
    return T(0); // unreachable
}

template<typename T>
g2o::VectorN<3, T> toPolar(g2o::VectorN<3, T> const& cart)
{
    T xySquared = (cart[0] * cart[0]) + (cart[1] * cart[1]);
    T radius = sqrt(xySquared + (cart[2] * cart[2]));
    g2o::VectorN<3, T> result;
    result[0] = radius;
    result[1] = atan2(sqrt(xySquared), cart[2]);
    result[2] = getPhi(cart[1], cart[0]);
    return result;
}

template<typename T>
g2o::VectorN<3, T> toCartesian(g2o::VectorN<3, T> const& sph) {
    g2o::VectorN<3, T> result;
    result[0] = sin(sph[1]) * cos(sph[2]) * sph[0];
    result[1] = sin(sph[1]) * sin(sph[2]) * sph[0];
    result[2] = cos(sph[1]) * sph[0];
    return result;
}



using namespace stella_vslam::optimize::internal;
using namespace stella_vslam::optimize::internal::se3;
using namespace std;

class euclidean_transform_vertex : public g2o::BaseVertex<6, g2o::bal::Vector6> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    euclidean_transform_vertex() {}

    virtual bool read(std::istream& /*is*/) {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual void setToOriginImpl() {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    virtual void oplusImpl(const double* update) {
        g2o::bal::Vector6::ConstMapType v(update, euclidean_transform_vertex::Dimension);
        _estimate += v;
    }
};

// templated version of stella_vslam::util::converter::to_skew_symmetric_mat
template<typename T>
g2o::MatrixN<3, T> to_skew_symmetric_mat(const g2o::VectorN<3, T>& vec) {
    g2o::MatrixN<3, T> skew;
    skew << T(0), -vec(2), vec(1),
        vec(2), T(0), -vec(0),
        -vec(1), vec(0), T(0);
    return skew;
}

template<typename T> void spdlogPrintMatrix(std::string const& text, g2o::MatrixN<3, T> const& matrix) {}    // Do nothing for complex (Jet) types
template<> void spdlogPrintMatrix<double>(std::string const& text, g2o::MatrixN<3, double> const& matrix)
{
    spdlog::info("{}", text);
    spdlog::info("{}", matrix);
}

template<typename T> void spdlogPrintVector(std::string const& text, g2o::VectorN<3, T> const& vector) {}    // Do nothing for complex (Jet) types
template<> void spdlogPrintVector<double>(std::string const& text, g2o::VectorN<3, double> const& vector)
{
    spdlog::info("{}", text);
    spdlog::info("{}", vector);
}

template<typename T> void spdlogPrintScalar(std::string const& text, T const& scalar) {}    // Do nothing for complex (Jet) types
template<> void spdlogPrintScalar<double>(std::string const& text, double const& scalar)
{
    spdlog::info("{} {}", text, scalar);
}
template<> void spdlogPrintScalar<ceres::Jet<double, 9>>(std::string const& text, ceres::Jet<double, 9> const& scalar)
{
    spdlog::info("{} {}", text, scalar);
}

// First template parameter is error dimension, second is the measurement type
class epipolar_sampson_error_edge
    : public g2o::BaseBinaryEdge<1, double, camera_intrinsics_vertex, euclidean_transform_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    epipolar_sampson_error_edge() {}

    static double const focal_length_conditioner;
    static double const translation_conditioner;
    static double const rotation_conditioner;

    double par;
    double cx;
    double cy;
    std::vector<Eigen::Matrix<double, 3, 1>> pts_1_double;
    std::vector<Eigen::Matrix<double, 3, 1>> pts_2_double;

    void create_jet_pts_from_double()
    {
        pts_1_jet9.resize(pts_1_double.size());
        for (int i = 0; i < pts_1_double.size(); ++i) {
            pts_1_jet9[i](0) = (ceres::Jet<double, 9>)pts_1_double[i](0);
            pts_1_jet9[i](1) = (ceres::Jet<double, 9>)pts_1_double[i](1);
            pts_1_jet9[i](2) = (ceres::Jet<double, 9>)pts_1_double[i](2);
        }
        pts_2_jet9.resize(pts_2_double.size());
        for (int i = 0; i < pts_2_double.size(); ++i) {
            pts_2_jet9[i](0) = (ceres::Jet<double, 9>)pts_2_double[i](0);
            pts_2_jet9[i](1) = (ceres::Jet<double, 9>)pts_2_double[i](1);
            pts_2_jet9[i](2) = (ceres::Jet<double, 9>)pts_2_double[i](2);
        }
    }

    std::vector<Eigen::Matrix<ceres::Jet<double, 9>, 3, 1>> pts_1_jet9;
    std::vector<Eigen::Matrix<ceres::Jet<double, 9>, 3, 1>> pts_2_jet9;

    template<typename T> std::vector<Eigen::Matrix<T, 3, 1>> const& pts_1() const; // Instantiated below for double and ceres::Jet<double, 9> types
    template<typename T> std::vector<Eigen::Matrix<T, 3, 1>> const& pts_2() const; // Instantiated below for double and ceres::Jet<double, 9> types

    virtual bool read(std::istream& /*is*/) {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    virtual bool write(std::ostream& /*os*/) const {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    /**
     * templatized function to compute the error as described in the comment above
     */
    template <typename T>
    bool operator()(const T* p_intrinsics, const T* p_transform, T* p_error) const {
        //typename g2o::VectorN<6, T>::ConstMapType v(p_transform);
        //typename g2o::VectorN<3, T>::ConstMapType intrinsics(p_intrinsics);

       // g2o::SE3Quat quat(transform);

        //g2o::VectorN<3, T> q = transform.template head<3>();

        Eigen::Quaternion<T> _r;

        g2o::VectorN<3, T> _t;
#if USE_TRANSLATION_DIRECTION
        _t[0] = T(1.);
        _t[1] = p_transform[0] / translation_conditioner;
        _t[2] = p_transform[1] / translation_conditioner;
        _t = toCartesian(_t);
#else
        for (int i = 0; i < 3; i++)
            _t[i] = p_transform[i] / translation_conditioner;
#endif

        // Based on g2o::SE3Quat constructor from a 6-vector
        for (int i = 0; i < 3; i++)
            _r.coeffs()(i) = p_transform[i + 3] / rotation_conditioner;
        
        _r.w() = T(0.);  // recover the positive w
        if (_r.norm() > 1.) {
            _r.normalize();
        }
        else {
            T w2 = T(1.) - _r.squaredNorm();
            _r.w() = (w2 < T(0.)) ? T(0.) : sqrt(w2);
        }

        // Convert to a rotation matrix
        auto R = _r.toRotationMatrix();

        //spdlogPrintMatrix("epipolar_sampson_error_edge::() R", R);
        //spdlogPrintVector("epipolar_sampson_error_edge::() t", _t);

        // Create fundamental matrix (Copy of fundamental_solver::create_F_21)
        T const& candidate_focal_length(p_intrinsics[0] / focal_length_conditioner); // Use a better focal_length from intrinsics function
        g2o::MatrixN<3, T> cam_matrix = camera_intrinsics_matrix(candidate_focal_length, (T)par, (T)cx, (T)cy);
        g2o::MatrixN<3, T> const& rot_21(R);
        g2o::VectorN<3, T> const& trans_21(_t);
        const g2o::MatrixN<3, T> trans_21_x = to_skew_symmetric_mat(trans_21);
        const g2o::MatrixN<3, T> E_21 = trans_21_x * rot_21;
        const g2o::MatrixN<3, T> F_21 = cam_matrix.transpose().inverse() * E_21 * cam_matrix.inverse();

        //spdlogPrintMatrix("epipolar_sampson_error_edge::() F-matrix", F_21);

        std::vector<bool> is_inlier_match; // result is ignored
        //unsigned int num_inliers = 
            compute_epipolar_sampson_error<T>(pts_1<T>(), pts_2<T>(), F_21, is_inlier_match, *p_error);

       // spdlogPrintScalar<T>("Error", *p_error);

        return true;
    }

    G2O_MAKE_AUTO_AD_FUNCTIONS
};

template<> std::vector<Eigen::Matrix<double, 3, 1>> const& epipolar_sampson_error_edge::pts_1<double>() const { return pts_1_double; }
template<> std::vector<Eigen::Matrix<double, 3, 1>> const& epipolar_sampson_error_edge::pts_2<double>() const { return pts_2_double; }
template<> std::vector<Eigen::Matrix<ceres::Jet<double, 9>, 3, 1>> const& epipolar_sampson_error_edge::pts_1<ceres::Jet<double, 9>>() const { return pts_1_jet9; }
template<> std::vector<Eigen::Matrix<ceres::Jet<double, 9>, 3, 1>> const& epipolar_sampson_error_edge::pts_2<ceres::Jet<double, 9>>() const { return pts_2_jet9; }

double const epipolar_sampson_error_edge::focal_length_conditioner = 0.000002;
double const epipolar_sampson_error_edge::rotation_conditioner = 1000.0;
#if USE_TRANSLATION_DIRECTION
double const epipolar_sampson_error_edge::translation_conditioner = 1.0;
#else
double const epipolar_sampson_error_edge::translation_conditioner = 1.0;
#endif

#if 0
class epipolar_point_matches_edge final : public g2o::BaseBinaryEdge<1, double, shot_vertex, camera_intrinsics_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        epipolar_point_matches_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;


    template <typename T>
    bool operator()(const T* shot, const T* intrinsics, T* error) const {
//        typename g2o::VectorN<2, T>::ConstMapType center(circle);
  //      const T& radius = circle[2];

    //    error[0] = (measurement().cast<T>() - center).norm() - radius;
        return true;
    }

   // void computeError() override;


//    bool depth_is_positive() const;

    //Vec2_t cam_project(const Vec3_t& pos_c) const;

    double fx_, fy_, cx_, cy_;

    G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
    //void linearizeOplus() override;

};


inline epipolar_point_matches_edge::epipolar_point_matches_edge()
    : g2o::BaseBinaryEdge<1, double, shot_vertex, camera_intrinsics_vertex>() {}

inline bool epipolar_point_matches_edge::read(std::istream& is) {
    // read the full measurement data.. todo
    //for (unsigned int i = 0; i < 2; ++i) {
        //is >> _measurement;
    //}
    for (int i = 0; i < information().rows(); ++i) {
        for (int j = i; j < information().cols(); ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

inline bool epipolar_point_matches_edge::write(std::ostream& os) const {
    // write out the full measurement data.. todo
    //for (unsigned int i = 0; i < 2; ++i) {
    //    os << measurement()(i) << " ";
    //}
    for (int i = 0; i < information().rows(); ++i) {
        for (int j = i; j < information().cols(); ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

//inline void epipolar_point_matches_edge::computeError() {
//    const auto v1 = static_cast<const shot_vertex*>(_vertices.at(0));
//    const auto v2 = static_cast<const camera_intrinsics_vertex*>(_vertices.at(1));
//    //const Vec2_t obs(_measurement);
//    _error(0, 0) = 12.9;//obs - cam_project(v1->estimate().map(v2->estimate()));
//}

//inline void epipolar_point_matches_edge::linearizeOplus() {
//    auto vj = static_cast<shot_vertex*>(_vertices.at(1));
//    const g2o::SE3Quat& cam_pose_cw = vj->shot_vertex::estimate();
//
//    auto vi = static_cast<landmark_vertex*>(_vertices.at(0));
//    const Vec3_t& pos_w = vi->landmark_vertex::estimate();
//    const Vec3_t pos_c = cam_pose_cw.map(pos_w);
//
//    const auto x = pos_c(0);
//    const auto y = pos_c(1);
//    const auto z = pos_c(2);
//    const auto z_sq = z * z;
//
//    const Mat33_t rot_cw = cam_pose_cw.rotation().toRotationMatrix();
//
//    _jacobianOplusXi(0, 0) = -fx_ * rot_cw(0, 0) / z + fx_ * x * rot_cw(2, 0) / z_sq;
//    _jacobianOplusXi(0, 1) = -fx_ * rot_cw(0, 1) / z + fx_ * x * rot_cw(2, 1) / z_sq;
//    _jacobianOplusXi(0, 2) = -fx_ * rot_cw(0, 2) / z + fx_ * x * rot_cw(2, 2) / z_sq;
//
//    _jacobianOplusXi(1, 0) = -fy_ * rot_cw(1, 0) / z + fy_ * y * rot_cw(2, 0) / z_sq;
//    _jacobianOplusXi(1, 1) = -fy_ * rot_cw(1, 1) / z + fy_ * y * rot_cw(2, 1) / z_sq;
//    _jacobianOplusXi(1, 2) = -fy_ * rot_cw(1, 2) / z + fy_ * y * rot_cw(2, 2) / z_sq;
//
//    _jacobianOplusXj(0, 0) = x * y / z_sq * fx_;
//    _jacobianOplusXj(0, 1) = -(1.0 + (x * x / z_sq)) * fx_;
//    _jacobianOplusXj(0, 2) = y / z * fx_;
//    _jacobianOplusXj(0, 3) = -1.0 / z * fx_;
//    _jacobianOplusXj(0, 4) = 0.0;
//    _jacobianOplusXj(0, 5) = x / z_sq * fx_;
//
//    _jacobianOplusXj(1, 0) = (1.0 + y * y / z_sq) * fy_;
//    _jacobianOplusXj(1, 1) = -x * y / z_sq * fy_;
//    _jacobianOplusXj(1, 2) = -x / z * fy_;
//    _jacobianOplusXj(1, 3) = 0.0;
//    _jacobianOplusXj(1, 4) = -1.0 / z * fy_;
//    _jacobianOplusXj(1, 5) = y / z_sq * fy_;
//}

//inline bool epipolar_point_matches_edge::depth_is_positive() const {
//    const auto v1 = static_cast<const shot_vertex*>(_vertices.at(1));
//    const auto v2 = static_cast<const landmark_vertex*>(_vertices.at(0));
//    return 0.0 < (v1->estimate().map(v2->estimate()))(2);
//}
//
//inline Vec2_t epipolar_point_matches_edge::cam_project(const Vec3_t& pos_c) const {
//    return { fx_ * pos_c(0) / pos_c(2) + cx_, fy_ * pos_c(1) / pos_c(2) + cy_ };
//}

#endif

// ////////////////////////////////////////////////////////////////////////////////////////////////////

// Copy of fundamental_solver::decompose
template<typename T> using eigen_alloc_vector = std::vector<T, Eigen::aligned_allocator<T>>;
bool fundamental_decompose(const Eigen::Matrix3d& F_21, const Eigen::Matrix3d& cam_matrix_1, const Eigen::Matrix3d& cam_matrix_2,
    eigen_alloc_vector<Eigen::Matrix3d>& init_rots, eigen_alloc_vector<Eigen::Vector3d>& init_transes)
{
    const Eigen::Matrix3d E_21 = cam_matrix_2.transpose() * F_21 * cam_matrix_1;
    stella_vslam::solve::essential_solver::decompose(E_21, init_rots, init_transes);
    return true;
}

// Copy of fundamental_solver::create_F_21
Eigen::Matrix3d fundamental_create_F_21(const Eigen::Matrix3d& rot_21,
                                        const Eigen::Vector3d& trans_21, const Eigen::Matrix3d& cam_matrix_1, const Eigen::Matrix3d& cam_matrix_2)
{
    //const Eigen::Matrix3d E_21 = stella_vslam::solve::essential_solver::create_E_21(rot_1w, trans_1w, rot_2w, trans_2w);
    //const Eigen::Matrix3d rot_21 = rot_2w * rot_1w.transpose();
    //const Eigen::Vector3d trans_21 = -rot_21 * trans_1w + trans_2w;
    const Eigen::Matrix3d trans_21_x = stella_vslam::util::converter::to_skew_symmetric_mat(trans_21);
    const Eigen::Matrix3d E_21 = trans_21_x * rot_21;

    return cam_matrix_2.transpose().inverse() * E_21 * cam_matrix_1.inverse();
}

// Copy of fundamental_solver::check_inliers
template<typename T>
unsigned int compute_epipolar_sampson_error(std::vector<Eigen::Matrix<T, 3, 1>> const& pts_1,
                                            std::vector<Eigen::Matrix<T, 3, 1>> const& pts_2,
                                            Eigen::Matrix<T, 3, 3> const& F_21,
                                            std::vector<bool>& is_inlier_match,
                                            T& cost)
{
    //const T sigma = T(1.0f);
    const T sigma = T(1000000.0f);

    unsigned int num_inliers = 0;
    const auto num_points = pts_1.size();

    // chi-squared value (p=0.05, n=2)
    const T chi_sq = T(5.991);

    is_inlier_match.resize(num_points);

    const T sigma_sq = sigma * sigma;

    cost = T(0.0);

    for (unsigned int i = 0; i < num_points; ++i) {

        Eigen::Matrix<T, 3, 1> const& pt_1(pts_1[i]);
        Eigen::Matrix<T, 3, 1> const& pt_2(pts_2[i]);

        // 2. Compute sampson error

        const Eigen::Matrix<T, 3, 1> F_21_pt_1 = F_21 * pt_1;
        const Eigen::Matrix<T, 1, 3> pt_2_F_21 = pt_2.transpose() * F_21;
        const T pt_2_F_21_pt_1 = pt_2_F_21 * pt_1;
        const T dist_sq = pt_2_F_21_pt_1 * pt_2_F_21_pt_1 / (F_21_pt_1.block<2, 1>(0, 0).squaredNorm() + pt_2_F_21.block<1, 2>(0, 0).squaredNorm());

        const T thr = chi_sq * sigma_sq;
        if (thr > dist_sq) {
            is_inlier_match.at(i) = true;
            cost += dist_sq;
            num_inliers++;
        }
        else {
            is_inlier_match.at(i) = false;
            cost += thr;
        }
    }

    // Convert cost to average pixel error as the cost should be an absolute error (not error squared)
    cost = sqrt(cost / T(num_points));

    return num_inliers;
}

// Copy of fundamental_solver::check_inliers
unsigned int fundamental_check_inliers_2(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
    const std::vector<std::pair<int, int>>& matches_12,
    const Eigen::Matrix3d& F_21, std::vector<bool>& is_inlier_match, float& cost)
{
    const auto num_points = matches_12.size();
    std::vector<Eigen::Matrix<double, 3, 1>> pts_1(num_points);
    std::vector<Eigen::Matrix<double, 3, 1>> pts_2(num_points);

    for (unsigned int i = 0; i < num_points; ++i) {
        const auto& keypt_1 = undist_keypts_1.at(matches_12.at(i).first);
        const auto& keypt_2 = undist_keypts_2.at(matches_12.at(i).second);

        // 1. Transform to homogeneous coordinates

        pts_1[i] = stella_vslam::util::converter::to_homogeneous(keypt_1.pt);
        pts_2[i] = stella_vslam::util::converter::to_homogeneous(keypt_2.pt);
    }

    double cost_double;
    unsigned int num_inliers = compute_epipolar_sampson_error<double>(pts_1, pts_2, F_21, is_inlier_match, cost_double);
    cost = (float)cost_double;

    return num_inliers;
}

// Copy of fundamental_solver::check_inliers
unsigned int fundamental_check_inliers(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
    const std::vector<std::pair<int, int>>& matches_12, 
    const Eigen::Matrix3d& F_21, std::vector<bool>& is_inlier_match, float& cost)
{
    const float sigma = 1.0f;

    unsigned int num_inliers = 0;
    const auto num_points = matches_12.size();

    // chi-squared value (p=0.05, n=2)
    constexpr float chi_sq = 5.991;

    is_inlier_match.resize(num_points);

    const float sigma_sq = sigma * sigma;

    cost = 0.0;

    for (unsigned int i = 0; i < num_points; ++i) {
        const auto& keypt_1 = undist_keypts_1.at(matches_12.at(i).first);
        const auto& keypt_2 = undist_keypts_2.at(matches_12.at(i).second);

        // 1. Transform to homogeneous coordinates

        const Eigen::Vector3d pt_1 = stella_vslam::util::converter::to_homogeneous(keypt_1.pt);
        const Eigen::Vector3d pt_2 = stella_vslam::util::converter::to_homogeneous(keypt_2.pt);

        // 2. Compute sampson error

        const Eigen::Vector3d F_21_pt_1 = F_21 * pt_1;
        const Eigen::Matrix<double, 1, 3> pt_2_F_21 = pt_2.transpose() * F_21;
        const double pt_2_F_21_pt_1 = pt_2_F_21 * pt_1;
        const double dist_sq = pt_2_F_21_pt_1 * pt_2_F_21_pt_1 / (F_21_pt_1.block<2, 1>(0, 0).squaredNorm() + pt_2_F_21.block<1, 2>(0, 0).squaredNorm());

        const float thr = chi_sq * sigma_sq;
        if (thr > dist_sq) {
            is_inlier_match.at(i) = true;
            cost += dist_sq;
            num_inliers++;
        }
        else {
            is_inlier_match.at(i) = false;
            cost += thr;
        }
    }

    return num_inliers;
}

template <typename T>
g2o::MatrixN<3, T> camera_intrinsics_matrix(T focal_length_x_pixels, T par, T cx, T cy)
{
    T fx = focal_length_x_pixels;
    T fy = focal_length_x_pixels * par;
    g2o::MatrixN<3, T> cam_matrix;
    //cam_matrix << fx, 0, 0, 0, fy, 0, cx, cy, 1.0;
    cam_matrix << fx, T(0), cx, T(0), fy, cy, T(0), T(0), T(1.0);

    return cam_matrix;
}

Eigen::Matrix3d f_decomp_recomp_test(double candidate_focal_length, double par, double cx, double cy, const Eigen::Matrix3d F_21)
{
    // Intrinsics matrix for the calculated focal length
    Eigen::Matrix3d cam_matrix = camera_intrinsics_matrix(candidate_focal_length, par, cx, cy);

    // Decompose F into the product of intrinsics, rotation, translation
    eigen_alloc_vector<Eigen::Matrix3d> init_rots;
    eigen_alloc_vector<Eigen::Vector3d> init_transes;
    bool ok_decompose = fundamental_decompose(F_21, cam_matrix, cam_matrix, init_rots, init_transes);
    if (!ok_decompose)
        return Eigen::Matrix3d();

    //spdlog::info("f_decomp_recomp_test R {}", init_rots[0]);
    //spdlog::info("f_decomp_recomp_test t {}", init_transes[0]);

    // Recompose
    int const n(init_rots.size());
    eigen_alloc_vector<Eigen::Matrix3d> recomposed_fs(n);
    for (int i = 0; i < n; ++i)
        recomposed_fs[i] = fundamental_create_F_21(init_rots[i], init_transes[i], cam_matrix, cam_matrix);

    return recomposed_fs[0];
}


double cost_for_focal_length(double candidate_focal_length, double par, double cx, double cy, const Eigen::Matrix3d F_21,
    const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
    std::vector<std::pair<int, int>> const& matches_12)
{
    // Intrinsics matrix for the calculated focal length
    Eigen::Matrix3d cam_matrix = camera_intrinsics_matrix(candidate_focal_length, par, cx, cy);

    // Decompose F into the product of intrinsics, rotation, translation
    eigen_alloc_vector<Eigen::Matrix3d> init_rots;
    eigen_alloc_vector<Eigen::Vector3d> init_transes;
    bool ok_decompose = fundamental_decompose(F_21, cam_matrix, cam_matrix, init_rots, init_transes);
    if (!ok_decompose)
        return 9999999.9;

    // Recompose
    int const n(init_rots.size());
    eigen_alloc_vector<Eigen::Matrix3d> recomposed_fs(n);
    for (int i = 0; i < n; ++i)
        recomposed_fs[i] = fundamental_create_F_21(init_rots[i], init_transes[i], cam_matrix, cam_matrix);


    //// decompose-recompose sanity check
    //eigen_alloc_vector<Eigen::Matrix3d> init_rots_2;
    //eigen_alloc_vector<Eigen::Vector3d> init_transes_2;
    //bool ok_decompose_2 = fundamental_decompose(recomposed_fs[0], cam_matrix, cam_matrix, init_rots_2, init_transes_2);
    //int const n_2(init_rots_2.size());
    //eigen_alloc_vector<Eigen::Matrix3d> recomposed_fs_2(n_2);
    //for (int i = 0; i < n_2; ++i)
    //    recomposed_fs_2[i] = fundamental_create_F_21(init_rots_2[i], init_transes_2[i], cam_matrix, cam_matrix);



    // Calculate error from the recomposed fundamental matrix
    std::vector<float> cost_n(n);
    std::vector<int> inlier_count_n(n);
    for (int i = 0; i < n; ++i) {
        std::vector<bool> these_inlier_matches;
        inlier_count_n[i] = fundamental_check_inliers(undist_keypts_1, undist_keypts_2,
            matches_12,
            recomposed_fs[i], these_inlier_matches, cost_n[i]);
    }

    return cost_n[0];
}


bool focal_length_estimator::test(input_matches const& matches,
                            stella_vslam::camera::base* camera,
                            std::map<std::pair<int, int>, error_graph_metrics>* metrics)
{
    // Use all the input matches in the calculation, or only the input inlier matches (filter_on_input_inlier_matches = true)
    bool filter_on_input_inlier_matches(true);

    // Return early if auto focal length is turned off
    if (camera->autocalibration_parameters_.optimise_focal_length == false)
        return false;

    // Camera intrinsics basics
    double focal_length_x_pixels_camera, par, cx, cy;
    bool ok_intrincics = intrinsics_from_camera(camera, focal_length_x_pixels_camera, par, cx, cy);
    if (!ok_intrincics)
        return false;


    for (auto const& frame_pair_match : matches.frame_matches) {

        // Collect the data
        frame_pair_matches::frame_id frame_id_1 = frame_pair_match.frame_id_1;
        frame_pair_matches::frame_id frame_id_2 = frame_pair_match.frame_id_2;
        auto f_features_1 = matches.undist_keypts.find(frame_id_1);
        if (f_features_1==matches.undist_keypts.end())
            continue;
        auto f_features_2 = matches.undist_keypts.find(frame_id_2);
        if (f_features_2 == matches.undist_keypts.end())
            continue;
        const std::vector<cv::KeyPoint>& undist_keypts_1(f_features_1->second);
        const std::vector<cv::KeyPoint>& undist_keypts_2(f_features_2->second);
        const std::vector<std::pair<int, int>>& input_matches_12(frame_pair_match.unguided_matches_12);
        std::vector<bool> const& input_inlier_matches(frame_pair_match.guided_inlier_matches);
        Eigen::Matrix3d const& F_21(frame_pair_match.F_21);

        std::vector<std::pair<int, int>> matches_12;
        if (filter_on_input_inlier_matches) {
            for (int i = 0; i < input_matches_12.size(); ++i) {
                if (input_inlier_matches[i])
                    matches_12.push_back(input_matches_12[i]);
            }
        }
        else
            matches_12 = input_matches_12;

        // Calculate cost from input F
        float cost;
        std::vector<bool> comp_inlier_matches;
        fundamental_check_inliers(undist_keypts_1, undist_keypts_2,
            matches_12,
            F_21, comp_inlier_matches, cost);

        // Initial estimate of the focal length
        bool focal_length_from_f_only_is_stable;
        double focal_length_from_f_only = min_geometric_error_focal_length(F_21, camera, &focal_length_from_f_only_is_stable);

        std::set<double> candidates = candidateFocalLengthsOverFOVRange(1, 170, camera->cols_);
        std::map<double, double> focal_length_to_cost;
        //std::map<double, double> focal_length_to_inlier_count;
        //std::map<double, double> focal_length_to_error;

        for (auto const& candidate_focal_length : candidates) {

            double cost_for_candidate_focal_length = cost_for_focal_length(candidate_focal_length, par, cx, cy, F_21,
                undist_keypts_1, undist_keypts_2, matches_12);

            focal_length_to_cost[candidate_focal_length] = cost_for_candidate_focal_length;

        }

        auto cost_min = std::min_element(focal_length_to_cost.begin(), focal_length_to_cost.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

        if (metrics) {
            error_graph_metrics frame_metrics;

            frame_metrics.error_for_max_focal_length = focal_length_to_cost.rbegin()->second;
            frame_metrics.focal_length_at_min_error = cost_min->first;

            double delta_f(1);
            double error_plus = cost_for_focal_length(frame_metrics.focal_length_at_min_error + delta_f, par, cx, cy, F_21,
                undist_keypts_1, undist_keypts_2, matches_12);
            double error_minus = cost_for_focal_length(frame_metrics.focal_length_at_min_error - delta_f, par, cx, cy, F_21,
                undist_keypts_1, undist_keypts_2, matches_12);
            frame_metrics.de_df_plus = -(error_plus - cost_min->second) / delta_f;
            frame_metrics.de_df_minus = (cost_min->second - error_minus) / delta_f;
            frame_metrics.error_min_value = cost_min->second;
            frame_metrics.min_error_percent_max_focal_error = 100.0 * frame_metrics.error_min_value / frame_metrics.error_for_max_focal_length;
            frame_metrics.focal_length_to_cost = focal_length_to_cost;

            frame_metrics.focal_length_from_f_only_is_stable = focal_length_from_f_only_is_stable;
            frame_metrics.focal_length_from_f_only = focal_length_from_f_only;

            (*metrics)[{frame_pair_match.frame_id_1, frame_pair_match.frame_id_2}] = frame_metrics;
        }

    }

    unsigned int num_iter(50);
    bool force_stop_flag(false);

    run_epipolar_optimisation(matches, camera, num_iter, &force_stop_flag);


    return true;
}

bool focal_length_estimator::test(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
    const std::vector<std::pair<int, int>>& input_matches_12, std::vector<bool> const& input_inlier_matches,
    const Eigen::Matrix3d F_21, stella_vslam::camera::base* camera,
    bool& focal_length_estimate_is_stable, bool& focal_length_changed)
{
    // Return early if auto focal length is turned off
    if (camera->autocalibration_parameters_.optimise_focal_length == false) {
        focal_length_estimate_is_stable = true;
        focal_length_changed = false;
        return false;
    }

    // Use all the input matches in the calculation, or only the input inlier matches (filter_on_input_inlier_matches = true)
    bool filter_on_input_inlier_matches(true);
    std::vector<std::pair<int, int>> matches_12;
    if (filter_on_input_inlier_matches) {
        for (int i = 0; i < input_matches_12.size(); ++i) {
            if (input_inlier_matches[i])
                matches_12.push_back(input_matches_12[i]);
        }
    }
    else
        matches_12 = input_matches_12;

    // Calculate cost from input F
    float cost;
    std::vector<bool> comp_inlier_matches;
    fundamental_check_inliers(undist_keypts_1, undist_keypts_2,
        matches_12,
        F_21, comp_inlier_matches, cost);



    // Camera intrinsics basics
    double focal_length_x_pixels_camera, par, cx, cy;
    bool ok_intrincics = intrinsics_from_camera(camera, focal_length_x_pixels_camera, par, cx, cy);
    if (!ok_intrincics)
        return false;

    // Initial estimate of the focal length
    bool focal_length_from_f_only_is_stable;
    double focal_length_from_f_only = min_geometric_error_focal_length(F_21, camera, &focal_length_from_f_only_is_stable);

    std::set<double> candidates = candidateFocalLengthsOverFOVRange(1, 170, camera->cols_);
    std::map<double, double> focal_length_to_cost;
    std::map<double, double> focal_length_to_inlier_count;
    std::map<double, double> focal_length_to_error;

    for (auto const& candidate_focal_length : candidates) {

        //double error2 = error_for_focal_length(F_21, camera, candidate_focal_length);
        //double error = error_for_focal_length(F_21, candidate_focal_length, par, cx, cy);


//        focal_length_to_error[candidate_focal_length] = error;

#if 1
        double cost_for_candidate_focal_length = cost_for_focal_length(candidate_focal_length, par, cx, cy, F_21,
            undist_keypts_1, undist_keypts_2, matches_12);

        focal_length_to_cost[candidate_focal_length] = cost_for_candidate_focal_length;

#else
        // Intrinsics matrix for the calculated focal length
        Eigen::Matrix3d cam_matrix = camera_intrinsics_matrix(candidate_focal_length, par, cx, cy);

        // Decompose F into the product of intrinsics, rotation, translation
        eigen_alloc_vector<Eigen::Matrix3d> init_rots;
        eigen_alloc_vector<Eigen::Vector3d> init_transes;
        bool ok_decompose = fundamental_decompose(F_21, cam_matrix, cam_matrix, init_rots, init_transes);

        // Recompose
        int const n(init_rots.size());
        eigen_alloc_vector<Eigen::Matrix3d> recomposed_fs(n);
        for (int i = 0; i < n; ++i)
            recomposed_fs[i] = fundamental_create_F_21(init_rots[i], init_transes[i], cam_matrix, cam_matrix);


        //// decompose-recompose sanity check
        //eigen_alloc_vector<Eigen::Matrix3d> init_rots_2;
        //eigen_alloc_vector<Eigen::Vector3d> init_transes_2;
        //bool ok_decompose_2 = fundamental_decompose(recomposed_fs[0], cam_matrix, cam_matrix, init_rots_2, init_transes_2);
        //int const n_2(init_rots_2.size());
        //eigen_alloc_vector<Eigen::Matrix3d> recomposed_fs_2(n_2);
        //for (int i = 0; i < n_2; ++i)
        //    recomposed_fs_2[i] = fundamental_create_F_21(init_rots_2[i], init_transes_2[i], cam_matrix, cam_matrix);



        // Calculate error from the recomposed fundamental matrix
        std::vector<float> cost_n(n);
        std::vector<int> inlier_count_n(n);
        for (int i = 0; i < n; ++i) {
            std::vector<bool> these_inlier_matches;
            inlier_count_n[i] = fundamental_check_inliers(undist_keypts_1, undist_keypts_2,
                matches_12,
                recomposed_fs[i], these_inlier_matches, cost_n[i]);
        }

        focal_length_to_cost[candidate_focal_length] = cost_n[0];
        focal_length_to_inlier_count[candidate_focal_length] = inlier_count_n[0];
#endif


    }

    //auto error_min = std::min_element(focal_length_to_error.begin(), focal_length_to_error.end(),
    //    [](const auto& a, const auto& b) { return a.second < b.second; });
    //double focal_length_x_pixels_at_min_error = error_min->first;

    auto cost_min = std::min_element(focal_length_to_cost.begin(), focal_length_to_cost.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });

    if (metrics::initialisation_debug().active()) {

        double error_for_max_focal_length = focal_length_to_cost.rbegin()->second;
        double focal_length_x_pixels_2 = cost_min->first;

        double delta_f(1);
        double error_plus = cost_for_focal_length(focal_length_x_pixels_2 + delta_f, par, cx, cy, F_21,
            undist_keypts_1, undist_keypts_2, matches_12);
        double error_minus = cost_for_focal_length(focal_length_x_pixels_2 - delta_f, par, cx, cy, F_21,
            undist_keypts_1, undist_keypts_2, matches_12);
        double de_df_plus = -(error_plus - cost_min->second) / delta_f;
        double de_df_minus = (cost_min->second - error_minus) / delta_f;
        double error_min_value = cost_min->second;
        double min_error_percent_max_focal_error = 100.0 * error_min_value / error_for_max_focal_length;

        metrics::initialisation_debug().submit_fundamental_decomp_debugging(error_for_max_focal_length,
            error_min_value,
            focal_length_x_pixels_2,
            de_df_plus,
            de_df_minus,
            min_error_percent_max_focal_error,
            focal_length_to_cost);

    }

    // Inputs: focal length, set of [points matches, F matrix]
    // 1) Take F-matrix, decompose (using focal length estimate) into Rt (fundamental_solver::decompose)
    // 2) Calculate cost (fundamental_solver::check_inliers) (fundamental_solver::create_F_21)
    // 3) Optimisation 
    //       vertex: focal length [1-param]
    //       vertex: Rt [number of pairs * 5-param]
    //       bi-edge: set of point match (x_1, x_2) to give  epipolar error (fundamental_solver::check_inliers)
    // 


    //g2o::SE3Quat - 6 parameter Rt
    //    Se3Quat::fromMinimalVector()

    focal_length_estimate_is_stable = focal_length_from_f_only_is_stable;
    focal_length_changed = focal_length_estimate_is_stable;

    if (focal_length_estimate_is_stable) {
        bool set_f_ok = stella_vslam_bfx::set_camera_focal_length_x_pixels(camera, focal_length_from_f_only);
        return set_f_ok;
    }
    return false;
}

bool focal_length_estimator::test_v2(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
    const std::vector<std::pair<int, int>>& input_matches_12, std::vector<bool> const& input_inlier_matches,
    const Eigen::Matrix3d F_21, stella_vslam::camera::base* camera,
    bool& focal_length_estimate_is_stable, bool& focal_length_changed)
{
    frame_pair_matches::frame_id frame_id_1(0.5); // arbitrary
    frame_pair_matches::frame_id frame_id_2(1.1); // arbitrary
    input_matches matches;
    matches.undist_keypts[frame_id_1] = undist_keypts_1;
    matches.undist_keypts[frame_id_2] = undist_keypts_2;
    frame_pair_matches frame_matches;
    frame_matches.frame_id_1 = frame_id_1;
    frame_matches.frame_id_2 = frame_id_2;
    frame_matches.unguided_matches_12 = input_matches_12; // All unguided matches
    frame_matches.guided_inlier_matches = input_inlier_matches; // Input inliers to the f-matrix
    frame_matches.F_21 = F_21; // fundamental matrix calculated during guided matching
    matches.frame_matches.push_back(frame_matches);





    std::map<std::pair<int, int>, error_graph_metrics> test_metrics;
    bool result_ok = test(matches, camera, &test_metrics);

    auto f_metrics = test_metrics.find({frame_id_1, frame_id_2});
    if (!result_ok || f_metrics == test_metrics.end()) {
        focal_length_estimate_is_stable = true;
        focal_length_changed = false;
        return false;
    }

    error_graph_metrics const& metrics(f_metrics->second);

    if (metrics::initialisation_debug().active()) {

        metrics::initialisation_debug().submit_fundamental_decomp_debugging(metrics.error_for_max_focal_length,
            metrics.error_min_value,
            metrics.focal_length_at_min_error,
            metrics.de_df_plus,
            metrics.de_df_minus,
            metrics.min_error_percent_max_focal_error,
            metrics.focal_length_to_cost);

    }

    focal_length_estimate_is_stable = metrics.focal_length_from_f_only_is_stable;
    focal_length_changed = focal_length_estimate_is_stable;

    if (focal_length_estimate_is_stable) {
        bool set_f_ok = stella_vslam_bfx::set_camera_focal_length_x_pixels(camera, metrics.focal_length_at_min_error);
        return set_f_ok;
    }

    return false;
}

   //g2o: Vertices: Fundamental matrices
   //      Edges : tri - edge, cost = funct(F1, F2, F3)



/*


To avoid illconditioned
points, we do not consider points where the two
transfer lines are nearly parallel or when the transfer lines
lay near the epipole.

The latter scenario can be checked by
examining the norm of the transfer line.Since the epipole
is in the null space of Fij, the norm of the transfer line will
be very small when it is near the epipole.


It should be noted that if the three camera centers are
collinear then there is a one - parameter family of planes
containing the three cameras and thus the trifocal plane is
ambiguous.We explicitly avoid this scenario by removing
collinear triplets where the epipoles are equal.

Similar to[9], we
first select the maximum spanning tree G′ = GMST where
edge weights are the number of inliers from fundamental
matrix estimation between two views then find all edges
ET ∈ E that, if added to G′ would create a triplet in the
graph(i.e., a loop of size 3) as show in Figure 3. Among
the edges in ET we select a set of “good” edges EG that
have a triplet projection error less than (see Appendix A)
and add these to the graph.

*/

using namespace stella_vslam;

template<class RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end)
{
    if (begin == end)
        return 0;
    std::size_t size = end - begin;
    std::size_t middleIdx = size / 2;
    RandAccessIter target = begin + middleIdx;
    std::nth_element(begin, target, end);

    if (size % 2 != 0) { //Odd number of elements
        return *target;
    }
    else { //Even number of elements
        double a = *target;
        RandAccessIter targetNeighbor = target - 1;
        std::nth_element(begin, targetNeighbor, end);
        return (a + *targetNeighbor) / 2.0;
    }
}

template<typename T>
T median(const std::vector<T>& data)
{
    std::vector<T> data_copy(data);
    return median(data_copy.begin(), data_copy.end());
}
// Instantiate for double
template double median<double>(const std::vector<double>& data);

// NB: c++20 has std::lerp, which can replace this 
double naive_lerp(double a, double b, double t) {
    return a + t * (b - a);
}

// e.g. auto quartiles = quantile<double>(vec, { 0.25, 0.5, 0.75 });
template<typename T>
std::vector<T> quantile(const std::vector<T>& data, const std::vector<T>& probs)
{
    if (data.empty())
        return std::vector<T>();

    if (data.size()==1)
        return { data[0] };

    std::vector<T> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    std::vector<T> quantiles;

    for (size_t i = 0; i < probs.size(); ++i)
    {
        T poi = naive_lerp(-0.5, sorted_data.size() - 0.5, probs[i]);

        int64_t left = std::max(int64_t(std::floor(poi)), int64_t(0));
        int64_t right = std::min(int64_t(std::ceil(poi)), int64_t(sorted_data.size() - 1));
        left = std::min(left, static_cast<int64_t>(sorted_data.size()) - 1);
        right = std::max(right, static_cast<int64_t>(0));

        T data_left = sorted_data.at(left);
        T data_right = sorted_data.at(right);

        T quantile = naive_lerp(data_left, data_right, poi - left);

        quantiles.push_back(quantile);
    }

    return quantiles;
}

// Instantiate for double
template std::vector<double> quantile<double>(const std::vector<double>& data, const std::vector<double>& probs);

double mean(std::vector<double> const& vec)
{
    if (vec.empty())
        return 0;
    double sum = 0.0;
    for (auto const& x : vec)
        sum += x;
    return sum / double(vec.size());
}

double standard_deviation(std::vector<double> const& vec)
{
    if (vec.empty())
        return 0;

    double m = mean(vec);

    double delta_sq_sum(0.0);
    for (auto const& x : vec) {
        double delta(x - m);
        delta_sq_sum += delta * delta;
    }

    return sqrt(delta_sq_sum / double(vec.size()));
}

std::vector<double> feature_motions(std::vector<cv::KeyPoint> const& undist_keypts_1,
                                    std::vector<cv::KeyPoint> const& undist_keypts_2,
                                    std::vector<std::pair<int, int>> const& unguided_matches_12,
                                    std::vector<bool> const* guided_inlier_matches)
{
    std::vector<double> result;
    for (int i = 0; i < unguided_matches_12.size(); ++i) {
        if (guided_inlier_matches && !(*guided_inlier_matches)[i])
            continue;
        const auto& keypt_1 = undist_keypts_1.at(unguided_matches_12.at(i).first);
        const auto& keypt_2 = undist_keypts_2.at(unguided_matches_12.at(i).second);
        auto const del(keypt_1.pt - keypt_2.pt);
        result.push_back(sqrt(del.x * del.x + del.y * del.y));
    }
    return result;
}

void run_epipolar_optimisation(input_matches const& matches,
                               stella_vslam::camera::base const* camera,
                               unsigned int num_iter,
                               bool* const force_stop_flag)
{
    //double deliberate_error_500(500.0); // to be removed
    //double deliberate_true(true);
    //double deliberate_1500(1500.0);

    const double focal_length_conditioner = epipolar_sampson_error_edge::focal_length_conditioner;
    const double rotation_conditioner = epipolar_sampson_error_edge::rotation_conditioner;
    const double translation_conditioner = epipolar_sampson_error_edge::translation_conditioner;



    // Camera par, cx, cy
    double focal_length_from_inputCamera_discarded, par, cx, cy;
    bool ok_intrincics = intrinsics_from_camera(camera, focal_length_from_inputCamera_discarded, par, cx, cy);
    if (!ok_intrincics)
        return;

    // 1. Construct and setup an optimizer
    using BlockSolverEpipolar = g2o::BlockSolverPL<6, 3>; // g2o::BlockSolverPL<6, 3>
    // PoseMatrixType = Eigen::Matrix<number_t, PoseDim, PoseDim, Eigen::ColMajor>

    // NB: optimiser deletes algorithm and all vertices and edges
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* algorithm(nullptr);
    // Block solver takes ownership of linear_solver
    // algorithm takes ownership of block_solver
#if 1
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::G2O_LINEAR_SOLVER_CLASS<g2o::BlockSolverX::PoseMatrixType>>();
//        auto linear_solver = g2o::make_unique<g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
#else
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::G2O_LINEAR_SOLVER_CLASS<BlockSolverEpipolar::PoseMatrixType>>();
        block_solver = g2o::make_unique<BlockSolverEpipolar>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
#endif
    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    unsigned int vtx_id(0);

    // 2. Calculate an initial estimate of the focal length from the frame match pair fundamental matrices
    std::vector<double> focal_length_estimates_from_f_only;
    for (auto const& frame_pair_match : matches.frame_matches) {
        bool focal_length_from_f_only_is_stable;
        double focal_length_from_f_only = min_geometric_error_focal_length(frame_pair_match.F_21, camera, &focal_length_from_f_only_is_stable);
        // NB: focal_length_from_f_only_is_stable is ignored
        focal_length_estimates_from_f_only.push_back(focal_length_from_f_only);

    }
    double median_focal_length_from_f_only = median(focal_length_estimates_from_f_only.begin(), focal_length_estimates_from_f_only.end());

//    median_focal_length_from_f_only = deliberate_1500; // remove

    // remove !!!!!!!!!!!!!!!!!!
    //median_focal_length_from_f_only += 400.0;

    //// temp
    //if (!matches.frame_matches.empty()) {
    //    Eigen::Matrix3d F_test = f_decomp_recomp_test(median_focal_length_from_f_only, par, cx, cy, matches.frame_matches.begin()->F_21);
    //    spdlogPrintMatrix("run_epipolar_optimisation() F-matrix", F_test);
    //    int y = 0;
    //}

    // 3. Setup the camera intrinsics (focal length) vertex and add it to the optimiser
    stella_vslam::optimize::internal::camera_intrinsics_vertex *camera_intrinsics_vtx = new stella_vslam::optimize::internal::camera_intrinsics_vertex;
    camera_intrinsics_vtx->setId(vtx_id++);
#ifdef USE_PADDED_CAMERA_INTRINSICS_VERTEX
    camera_intrinsics_vtx->setEstimate(stella_vslam::optimize::internal::camera_intrinsics_vertex_type(focal_length_conditioner *median_focal_length_from_f_only, 0.0, 0.0));
#else
    camera_intrinsics_vtx->setEstimate(focal_length_conditioner * median_focal_length_from_f_only);
#endif
    camera_intrinsics_vtx->setFixed(false);
    camera_intrinsics_vtx->setMarginalized(false); // "this node is marginalized out during the optimization"
    bool ok_add_camera_intrinsics_vtx = optimizer.addVertex(camera_intrinsics_vtx);
    if (!ok_add_camera_intrinsics_vtx)
        return;

   // int remove_me = optimizer.activeVertices().size();

    // Intrinsics matrix for the initial focal length estimate
    Eigen::Matrix3d cam_matrix = camera_intrinsics_matrix(median_focal_length_from_f_only, par, cx, cy);

    std::optional< Eigen::Matrix<double, 6, 1>> first_rt_vertex_initial_params;
    std::map<int, std::pair<frame_pair_matches::frame_id, frame_pair_matches::frame_id>> rt_vertex_id_to_frame_pair;
    std::map<int, euclidean_transform_vertex*> rt_vertex;
    std::map< std::pair<frame_pair_matches::frame_id, frame_pair_matches::frame_id>, epipolar_sampson_error_edge*> epipolar_edge_map; // map from frame pair to edge
    for (auto const& frame_pair_match : matches.frame_matches) {

        // 4. Create Rt vertices for each frame pair match
        // 
        // Decompose the f-matrix using the initial focal length estimate to get initial R t estimates for this match


        // Decompose F into the product of intrinsics, rotation, translation
        eigen_alloc_vector<Eigen::Matrix3d> init_rots;
        eigen_alloc_vector<Eigen::Vector3d> init_transes;
        bool ok_decompose = fundamental_decompose(frame_pair_match.F_21, cam_matrix, cam_matrix, init_rots, init_transes);
        if (!ok_decompose)
            continue;
        Eigen::Matrix3d const& rot = init_rots[0];
        Eigen::Vector3d const& trans = init_transes[0];

        Eigen::Quaternion<double> q(rot);
        Eigen::Matrix<double, 6, 1> params;

#if USE_TRANSLATION_DIRECTION
        g2o::VectorN<3, double> polar = toPolar(trans);
        params[0] = translation_conditioner * polar[1];
        params[1] = translation_conditioner * polar[2];
        params[2] = 0; // not used
#else
        params[0] = translation_conditioner * trans[0];
        params[1] = translation_conditioner * trans[1];
        params[2] = translation_conditioner * trans[2];
#endif
        params[3] = rotation_conditioner * q.x();
        params[4] = rotation_conditioner * q.y();
        params[5] = rotation_conditioner * q.z();

        if (!first_rt_vertex_initial_params)
            first_rt_vertex_initial_params = params;

#if 0
        spdlog::info("run_epipolar_optimisation initial R {}", rot);
        spdlog::info("run_epipolar_optimisation initial t {}", trans);

        // check - code from epiolar_sampson_error_edge
        //auto p = g2o::SE3Quat{ rot, trans };
        Eigen::Quaternion<double> _r_check;
        for (int i = 0; i < 3; i++) {
            _r_check.coeffs()(i) = params[i + 3];
        }
        _r_check.w() = 0.;  // recover the positive w
        if (_r_check.norm() > 1.) {
            _r_check.normalize();
        }
        else {
            double w2 = 1. - _r_check.squaredNorm();
            _r_check.w() = (w2 < 0.) ? 0. : sqrt(w2);
        }
        auto R_check = _r_check.toRotationMatrix();
        spdlog::info("run_epipolar_optimisation initial R_check {}", R_check);
#endif

        int rt_vertex_id = vtx_id++;
        rt_vertex_id_to_frame_pair[rt_vertex_id] = { frame_pair_match.frame_id_1, frame_pair_match.frame_id_2 };

        euclidean_transform_vertex* pair_rt_vertex = new euclidean_transform_vertex();
        rt_vertex[rt_vertex_id] = pair_rt_vertex;
        pair_rt_vertex->setId(rt_vertex_id);


        pair_rt_vertex->setEstimate(params);
        pair_rt_vertex->setFixed(false);

        bool ok_add_pair_rt_vertex = optimizer.addVertex(pair_rt_vertex);
        if (!ok_add_pair_rt_vertex)
            continue;

        // 5. Create an epipolar error edge for each frame pair match

        epipolar_sampson_error_edge* new_edge = new epipolar_sampson_error_edge();
        epipolar_edge_map[{frame_pair_match.frame_id_1, frame_pair_match.frame_id_2}] = new_edge;

        new_edge->par = par;
        new_edge->cx = cx;
        new_edge->cy = cy;

        // Fill in the point matches
        auto f_undist_keypts_1 = matches.undist_keypts.find(frame_pair_match.frame_id_1);
        if (f_undist_keypts_1 == matches.undist_keypts.end())
            continue;
        auto f_undist_keypts_2 = matches.undist_keypts.find(frame_pair_match.frame_id_2);
        if (f_undist_keypts_2 == matches.undist_keypts.end())
            continue;
        std::vector<cv::KeyPoint> const& undist_keypts_1(f_undist_keypts_1->second);
        std::vector<cv::KeyPoint> const& undist_keypts_2(f_undist_keypts_2->second);
        for (int i = 0; i < frame_pair_match.unguided_matches_12.size(); ++i) {
            if (frame_pair_match.guided_inlier_matches[i]) {
                new_edge->pts_1_double.push_back(stella_vslam::util::converter::to_homogeneous(undist_keypts_1[frame_pair_match.unguided_matches_12[i].first].pt));
                new_edge->pts_2_double.push_back(stella_vslam::util::converter::to_homogeneous(undist_keypts_2[frame_pair_match.unguided_matches_12[i].second].pt));
            }
        }
        new_edge->create_jet_pts_from_double();


        new_edge->setVertex(0, camera_intrinsics_vtx);
        new_edge->setVertex(1, pair_rt_vertex);
        new_edge->setInformation(g2o::MatrixN<1>::Identity());
        new_edge->setMeasurement(0);

        bool ok_add_edge = optimizer.addEdge(new_edge);
        if (!ok_add_edge)
            continue;
    }


    // 6. Perform optimization

    optimizer.setComputeBatchStatistics(true);
    optimizer.initializeOptimization();

    //const g2o::SparseOptimizer::VertexContainer& aV = optimizer.activeVertices();
    //const g2o::SparseOptimizer::EdgeContainer& aE = optimizer.activeEdges();
    
    //double aC = optimizer.activeChi2();
    //double aRC = optimizer.activeRobustChi2();
    //optimizer.computeActiveErrors();
    //double aC2 = optimizer.activeChi2();
    //double aRC2 = optimizer.activeRobustChi2();


    // Value test
#if 0
    for (double offset = -500.0; offset < 501.0; offset += 10.0) {

        double i = median_focal_length_from_f_only + offset;
        camera_intrinsics_vtx->setEstimate(stella_vslam::optimize::internal::camera_intrinsics_vertex_type(focal_length_conditioner* i, 0.0, 0.0));
        optimizer.computeActiveErrors();
        double chi2_i = optimizer.activeChi2();
        double rms_i = sqrt(optimizer.activeChi2() / (double)optimizer.activeEdges().size());
        spdlog::info("focal length {} average pixel error {} chi2 {}", i, rms_i, chi2_i);
    }
    // set the corrrect initial value
    camera_intrinsics_vtx->setEstimate(stella_vslam::optimize::internal::camera_intrinsics_vertex_type(focal_length_conditioner*median_focal_length_from_f_only, 0.0, 0.0));
#endif

    // chi squared = Sum { (observed_i-expected_i)^2 / expected_i }
    // g2o chi_squared = sum_over_edges(edge_error dot (edge_information * edge_error)
    int pre_edges(-1);
    double pre_rms(-1), pre_rms_robust(-1), pre_chi2(-1), pre_chi2_robust(-1);
    if (true) { // should be removed - 
        optimizer.computeActiveErrors();
        pre_rms = sqrt(optimizer.activeChi2() / (double)optimizer.activeEdges().size());
        pre_rms_robust = sqrt(optimizer.activeRobustChi2() / (double)optimizer.activeEdges().size());
        pre_chi2 = optimizer.activeChi2();
        pre_chi2_robust = optimizer.activeRobustChi2();
        pre_edges = optimizer.activeEdges().size();
        //spdlog::info("              before epipolar opt chi2 {} robust-chi2 {} edges {} rms {} robust-rms {}",
        //    pre_chi2, pre_chi2_robust, pre_edges, pre_rms, pre_rms_robust);
    }

    double focal_length_before = camera_intrinsics_vtx->estimate()[0] / focal_length_conditioner;

    bool ok = optimizer.optimize(num_iter);
    if (!ok)
        return; // error

    double rms = sqrt(optimizer.activeChi2() / (double)optimizer.activeEdges().size());


    //double standard_deviation(std::vector<double> const& vec)

    std::vector<double> m_ep_dedf;
    std::vector<double> m_ep_dedtu;
    std::vector<double> m_ep_dedtv;
    std::vector<double> m_ep_dedrx;
    std::vector<double> m_ep_dedry;
    std::vector<double> m_ep_dedrz;
    g2o::JacobianWorkspace& jacobianWorkspace = optimizer.jacobianWorkspace();
    for (int k = 0; k < static_cast<int>(optimizer.activeEdges().size()); ++k) {
        g2o::OptimizableGraph::Edge* e = optimizer.activeEdges()[k];
        e->linearizeOplus(jacobianWorkspace);  // jacobian of the nodes' oplus (manifold)
        double* j0 = jacobianWorkspace.workspaceForVertex(0);
        double* j1 = jacobianWorkspace.workspaceForVertex(1);

        m_ep_dedf.push_back(j0[0] * focal_length_conditioner);
        m_ep_dedtu.push_back(j1[0] * translation_conditioner);
        m_ep_dedtv.push_back(j1[1] * translation_conditioner);
        m_ep_dedrx.push_back(j1[3] * rotation_conditioner);
        m_ep_dedry.push_back(j1[4] * rotation_conditioner);
        m_ep_dedrz.push_back(j1[5] * rotation_conditioner);
    }
    //for (int i=0; i< focal_length_gradients.size(); ++i)
    //    spdlog::info("de/df [{}] = {}", i, m_ep_dedf[i]);

    double dedf = std::abs(mean(m_ep_dedf));
    double dedtu = std::abs(mean(m_ep_dedtu));
    double dedtv = std::abs(mean(m_ep_dedtv));
    double dedrx = std::abs(mean(m_ep_dedrx));
    double dedry = std::abs(mean(m_ep_dedry));
    double dedrz = std::abs(mean(m_ep_dedrz));

    double std_dev_dedf = standard_deviation(m_ep_dedf);
    double std_dev_dedtu = standard_deviation(m_ep_dedtu);
    double std_dev_dedtv = standard_deviation(m_ep_dedtv);
    double std_dev_dedrx = standard_deviation(m_ep_dedrx);
    double std_dev_dedry = standard_deviation(m_ep_dedry);
    double std_dev_dedrz = standard_deviation(m_ep_dedrz);

    metrics::initialisation_debug().submit_epipolar_estimator_debugging(rms, focal_length_before, camera_intrinsics_vtx->estimate()[0] / focal_length_conditioner,
        dedf, dedtu, dedtv, dedrx, dedry, dedrz,
        std_dev_dedf, std_dev_dedtu, std_dev_dedtv, std_dev_dedrx, std_dev_dedry, std_dev_dedrz);

    //std::map< std::pair<frame_pair_matches::frame_id, frame_pair_matches::frame_id>, double> epipolar_edge_hessian_map; // map from frame pair to edge
    //for (auto const& edge : epipolar_edge_hessian_map)
    //    epipolar_edge_hessian_map

    spdlog::info("focal_length_conditioner {}", focal_length_conditioner);
    spdlog::info("average pixel error {} -> {}", pre_rms, rms);
    spdlog::info("chi squared {} -> {}", pre_chi2, optimizer.activeChi2());
    spdlog::info("focal length {} -> {}", focal_length_before, camera_intrinsics_vtx->estimate()[0]/ focal_length_conditioner);
    if (first_rt_vertex_initial_params && !rt_vertex.empty()) {
        spdlog::info("rt   {}", first_rt_vertex_initial_params.value().transpose());
        spdlog::info("  -> {}", rt_vertex.begin()->second->estimate().transpose());
    }

    //g2o::BatchStatisticsContainer& stats = optimizer.batchStatistics();
    //for (int i = 0; i < stats.size(); ++i) {
    //    if (stats[i].iteration < 0)
    //        continue;
    //    auto const& stat(stats[i]);
    //    double gain = i == 0 ? 0 : (stats[i-1].chi2 - stats[i].chi2) / stats[i].chi2;
    //    double gain2 = i == 0 ? 0 : (sqrt(stats[i - 1].chi2) - sqrt(stats[i].chi2)) / sqrt(stats[i].chi2);
    //    spdlog::info("-> iter {} #vertices {} #edges {} chi2 {} gain {} {}", stat.iteration, stat.numVertices, stat.numEdges, stat.chi2, gain, gain2);
    //}

    if (force_stop_flag && *force_stop_flag) {
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

focal_length_estimator* focal_length_estimator::get_instance() {
    if (!instance)
        instance = new focal_length_estimator();
    return instance;
}

void focal_length_estimator::clear() {
    if (instance) {
        delete instance;
        instance = nullptr;
    }
}

bool focal_length_estimator::add_frame_pair(const std::vector<cv::KeyPoint>& undist_keypts_1,
                                            const std::vector<cv::KeyPoint>& undist_keypts_2,
                                            const std::vector<std::pair<int, int>>& input_matches_12,
                                            std::vector<bool> const& input_inlier_matches,
                                            const Eigen::Matrix3d F_21)
{
    frame_pair_matches::frame_id frame_id_1(current_frame_pair[0]);
    frame_pair_matches::frame_id frame_id_2(current_frame_pair[1]);

    // Add the points to m_matches, if they're not already there
    auto f_undist_keypts_1 = m_matches.undist_keypts.find(frame_id_1);
    if (f_undist_keypts_1 == m_matches.undist_keypts.end())
        m_matches.undist_keypts[frame_id_1] = undist_keypts_1;
    auto f_undist_keypts_2 = m_matches.undist_keypts.find(frame_id_2);
    if (f_undist_keypts_2 == m_matches.undist_keypts.end())
        m_matches.undist_keypts[frame_id_2] = undist_keypts_2;

#if 1
    // sanity check
    int count_pts_1(undist_keypts_1.size());
    int count_pts_2(undist_keypts_2.size());

    int max_match_pts_1 = std::max_element(input_matches_12.begin(), input_matches_12.end(),
        [](std::pair<int, int> const& lhs, std::pair<int, int> const& rhs) { return lhs.first < rhs.first; })->first;
    int max_match_pts_2 = std::max_element(input_matches_12.begin(), input_matches_12.end(),
        [](std::pair<int, int> const& lhs, std::pair<int, int> const& rhs) { return lhs.second < rhs.second; })->second;

    int count_pts_1b(m_matches.undist_keypts[frame_id_1].size());
    int count_pts_2b(m_matches.undist_keypts[frame_id_2].size());

    spdlog::info("-->frames {}-{} points {}-{} {}-{} matches {}-{}", current_frame_pair[0], current_frame_pair[1], count_pts_1, count_pts_2, count_pts_1b, count_pts_2b, max_match_pts_1, max_match_pts_2);
    if (max_match_pts_1 >= count_pts_1 || max_match_pts_2 >= count_pts_2)
        int yy = 0;
    if (count_pts_1 != count_pts_1b || count_pts_2 != count_pts_2b)
        int yyy = 0;
#endif


    // Create a new frame_pair_matches entry in m_matches
    frame_pair_matches frame_matches;
    frame_matches.frame_id_1 = frame_id_1;
    frame_matches.frame_id_2 = frame_id_2;
    frame_matches.unguided_matches_12 = input_matches_12; // All unguided matches
    frame_matches.guided_inlier_matches = input_inlier_matches; // Input inliers to the f-matrix
    frame_matches.F_21 = F_21; // fundamental matrix calculated during guided matching
    m_matches.frame_matches.push_back(frame_matches);

    // Calculate feature motion
    if (stella_vslam_bfx::metrics::initialisation_debug().active()) {
        std::vector<double> motions = feature_motions(undist_keypts_1, undist_keypts_2, input_matches_12, &input_inlier_matches);
        std::vector<double> feature_motion_quantiles = quantile(motions, { 0.25, 0.5, 0.75 });
        stella_vslam_bfx::metrics::initialisation_debug().submit_feature_motions(feature_motion_quantiles[0],
                                                                                 feature_motion_quantiles[1],
                                                                                 feature_motion_quantiles[2]);
    }

    return true;
}

bool focal_length_estimator::run_optimisation(stella_vslam::camera::base* camera,
                                              bool& focal_length_estimate_is_stable,
                                              bool& focal_length_changed)
{
    bool const current_frame_pair_only(true); // a test!!

    unsigned int num_iter(50);
    bool force_stop_flag(false);
    if (current_frame_pair_only) {
        input_matches latest_matches(m_matches);
        latest_matches.frame_matches.clear();
        auto f = std::find_if(m_matches.frame_matches.begin(), m_matches.frame_matches.end(),
            [&to_find = current_frame_pair](const frame_pair_matches& x) { return to_find == std::array<double, 2>({x.frame_id_1, x.frame_id_2}); });
        if (f != m_matches. frame_matches.end())
            latest_matches.frame_matches.push_back(*f);
        run_epipolar_optimisation(latest_matches, camera, num_iter, &force_stop_flag);
    }
    else
        run_epipolar_optimisation(m_matches, camera, num_iter, &force_stop_flag);

    // For now...
    focal_length_estimate_is_stable = false;
    focal_length_changed = false;


//    auto f_metrics = test_metrics.find({ frame_id_1, frame_id_2 });
//    if (!result_ok || f_metrics == test_metrics.end()) {
//        focal_length_estimate_is_stable = true;
//        focal_length_changed = false;
//        return false;
//    }
//
//    error_graph_metrics const& metrics(f_metrics->second);
//
//    if (metrics::initialisation_debug().active()) {
//
//        metrics::initialisation_debug().submit_fundamental_decomp_debugging(metrics.error_for_max_focal_length,
//            metrics.error_min_value,
//            metrics.focal_length_at_min_error,
//            metrics.de_df_plus,
//            metrics.de_df_minus,
//            metrics.min_error_percent_max_focal_error,
//            metrics.focal_length_to_cost);
//
//    }
//
//    focal_length_estimate_is_stable = metrics.focal_length_from_f_only_is_stable;
//    focal_length_changed = focal_length_estimate_is_stable;
//
//    if (focal_length_estimate_is_stable) {
//        bool set_f_ok = stella_vslam_bfx::set_camera_focal_length_x_pixels(camera, metrics.focal_length_at_min_error);
//        return set_f_ok;
//    }
//#endif
    return false;
}

} // namespace stella_vslam_bfx
