/** \file
 * \brief Definition of camera intrinsic parameters vertex classes
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/internal/landmark_vertex.h"
#include "stella_vslam/optimize/internal/se3/shot_vertex.h"
#include "stella_vslam/optimize/internal/bfx_camera_intrinsics_vertex.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

namespace g2o {


// This could be a simple using statement, but in multiple places
// _jacobianOplusXi and _jacobianOplusYi are used.
template<int D, typename E, typename VertexXi, typename VertexXj, typename VertexXk>
class BaseTertiaryEdge : public BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk> {
public:
    using VertexXiType = VertexXi;
    using VertexXjType = VertexXj;
    using VertexXkType = VertexXk;
    BaseTertiaryEdge()
        : BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>(){};

protected:
    typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>::template JacobianType<
        D, VertexXi::Dimension>& _jacobianOplusXi
        = std::get<0>(this->_jacobianOplus);
    typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>::template JacobianType<
        D, VertexXj::Dimension>& _jacobianOplusXj
        = std::get<1>(this->_jacobianOplus);
    typename BaseFixedSizedEdge<D, E, VertexXi, VertexXj, VertexXk>::template JacobianType<
        D, VertexXk::Dimension>& _jacobianOplusXk
        = std::get<2>(this->_jacobianOplus);
};

} // namespace g2o

namespace stella_vslam {
namespace optimize {
namespace internal {
namespace se3 {

 /** 
 * \brief Reprojection error based on 3 vertices: Camera, 3DPoint, Camera intrinsics
 *
 * Used mono_perspective_reproj_edge as a base. Some functions are identical: read(), write(), depth_is_positive()
 */
class bfx_mono_perspective_reproj_tri_edge final : public g2o::BaseTertiaryEdge<2, Vec2_t, landmark_vertex, shot_vertex, bfx_camera_intrinsics_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bfx_mono_perspective_reproj_tri_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;

    bool depth_is_positive() const;

    Vec2_t cam_project(const Vec3_t& pos_c, double focal_length_x_pix) const;

    double par_, cx_, cy_;
};

inline bfx_mono_perspective_reproj_tri_edge::bfx_mono_perspective_reproj_tri_edge()
    : g2o::BaseTertiaryEdge<2, Vec2_t, landmark_vertex, shot_vertex, bfx_camera_intrinsics_vertex>() {}

inline bool bfx_mono_perspective_reproj_tri_edge::read(std::istream& is) {
    for (unsigned int i = 0; i < 2; ++i) {
        is >> _measurement(i);
    }
    is >> information()(0, 0);
    is >> information()(0, 1);
    is >> information()(1, 1);
    information()(1, 0) = information()(0, 1);
    return true;
}

inline bool bfx_mono_perspective_reproj_tri_edge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 2; ++i) {
        os << measurement()(i) << " ";
    }
    os << " " << information()(0, 0);
    os << " " << information()(0, 1); // == information()(1, 0)
    os << " " << information()(1, 1);
    return os.good();
}

inline void bfx_mono_perspective_reproj_tri_edge::computeError() {
    const auto v1 = static_cast<const shot_vertex*>(_vertices.at(1));
    const auto v2 = static_cast<const landmark_vertex*>(_vertices.at(0));
    const auto v3 = static_cast<const bfx_camera_intrinsics_vertex*>(_vertices.at(2));
    const Vec2_t obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()), v3->estimate());
}

inline void bfx_mono_perspective_reproj_tri_edge::linearizeOplus() {
    auto vj = static_cast<shot_vertex*>(_vertices.at(1));
    const g2o::SE3Quat& cam_pose_cw = vj->shot_vertex::estimate();

    auto vi = static_cast<landmark_vertex*>(_vertices.at(0));
    const Vec3_t& pos_w = vi->landmark_vertex::estimate();
    const Vec3_t pos_c = cam_pose_cw.map(pos_w);

    auto v3 = static_cast<bfx_camera_intrinsics_vertex*>(_vertices.at(2));
    double fx = v3->bfx_camera_intrinsics_vertex::estimate();
    double fy = par_ * fx; 

    const auto x = pos_c(0);
    const auto y = pos_c(1);
    const auto z = pos_c(2);
    const auto z_sq = z * z;

    const Mat33_t rot_cw = cam_pose_cw.rotation().toRotationMatrix();

    _jacobianOplusXi(0, 0) = -fx * rot_cw(0, 0) / z + fx * x * rot_cw(2, 0) / z_sq;
    _jacobianOplusXi(0, 1) = -fx * rot_cw(0, 1) / z + fx * x * rot_cw(2, 1) / z_sq;
    _jacobianOplusXi(0, 2) = -fx * rot_cw(0, 2) / z + fx * x * rot_cw(2, 2) / z_sq;

    _jacobianOplusXi(1, 0) = -fy * rot_cw(1, 0) / z + fy * y * rot_cw(2, 0) / z_sq;
    _jacobianOplusXi(1, 1) = -fy * rot_cw(1, 1) / z + fy * y * rot_cw(2, 1) / z_sq;
    _jacobianOplusXi(1, 2) = -fy * rot_cw(1, 2) / z + fy * y * rot_cw(2, 2) / z_sq;

    _jacobianOplusXj(0, 0) = x * y / z_sq * fx;
    _jacobianOplusXj(0, 1) = -(1.0 + (x * x / z_sq)) * fx;
    _jacobianOplusXj(0, 2) = y / z * fx;
    _jacobianOplusXj(0, 3) = -1.0 / z * fx;
    _jacobianOplusXj(0, 4) = 0.0;
    _jacobianOplusXj(0, 5) = x / z_sq * fx;

    _jacobianOplusXj(1, 0) = (1.0 + y * y / z_sq) * fy;
    _jacobianOplusXj(1, 1) = -x * y / z_sq * fy;
    _jacobianOplusXj(1, 2) = -x / z * fy;
    _jacobianOplusXj(1, 3) = 0.0;
    _jacobianOplusXj(1, 4) = -1.0 / z * fy;
    _jacobianOplusXj(1, 5) = y / z_sq * fy;

    _jacobianOplusXk(0, 0) = -x / z;
    _jacobianOplusXk(1, 0) = -(par_ * y) / z;
}

inline bool bfx_mono_perspective_reproj_tri_edge::depth_is_positive() const {
    const auto v1 = static_cast<const shot_vertex*>(_vertices.at(1));
    const auto v2 = static_cast<const landmark_vertex*>(_vertices.at(0));
    return 0.0 < (v1->estimate().map(v2->estimate()))(2);
}

inline Vec2_t bfx_mono_perspective_reproj_tri_edge::cam_project(const Vec3_t& pos_c, double focal_length_x_pix) const {
    return {focal_length_x_pix * pos_c(0) / pos_c(2) + cx_, focal_length_x_pix * par_ * pos_c(1) / pos_c(2) + cy_};
}


} // namespace se3
} // namespace internal
} // namespace optimize
} // namespace stella_vslam

