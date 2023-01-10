/** \file
 * \brief Definition of camera intrinsic parameters vertex classes
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include "stella_vslam/type.h"

#include <g2o/core/base_vertex.h>

namespace stella_vslam {
namespace optimize {
namespace internal {
/** 
 * \brief Camera intrinsics class
 *
 * Currently 1 value, which is focal length in x direction pixels
 * 
 * Used landmark_vertex as a starting point
 */
class camera_intrinsics_vertex_1 final : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    camera_intrinsics_vertex_1();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update) override;
};

inline camera_intrinsics_vertex_1::camera_intrinsics_vertex_1()
    : g2o::BaseVertex<1, double>() {}

inline bool camera_intrinsics_vertex_1::read(std::istream& is) {
    is >> _estimate;
    return true;
}

inline bool camera_intrinsics_vertex_1::write(std::ostream& os) const {
    const double focalLengthXPixels = estimate();
    os << focalLengthXPixels << " ";
    return os.good();
}

inline void camera_intrinsics_vertex_1::setToOriginImpl() {
    _estimate = 0.0; // g2o says "sets the node to the origin (used in the multilevel stuff)"
}

inline void camera_intrinsics_vertex_1::oplusImpl(const double* update) {
    _estimate += *update;
}

/** 
 * \brief Camera intrinsics class 2-vector
 *
 * Currently 1 value, which is focal length in x direction pixels
 * 
 * Used landmark_vertex as a starting point (search '3', replace with '2')
 */
class camera_intrinsics_vertex_2 final : public g2o::BaseVertex<3, Vec3_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    camera_intrinsics_vertex_2();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update) override;
};

inline camera_intrinsics_vertex_2::camera_intrinsics_vertex_2()
    : g2o::BaseVertex<3, Vec3_t>() {}

inline bool camera_intrinsics_vertex_2::read(std::istream& is) {
    Vec3_t lv;
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _estimate(i);
    }
    return true;
}

inline bool camera_intrinsics_vertex_2::write(std::ostream& os) const {
    const Vec3_t pos_w = estimate();
    for (unsigned int i = 0; i < 3; ++i) {
        os << pos_w(i) << " ";
    }
    return os.good();
}

inline void camera_intrinsics_vertex_2::setToOriginImpl() {
    _estimate.fill(0);  // g2o says "sets the node to the origin (used in the multilevel stuff)"
}

inline void camera_intrinsics_vertex_2::oplusImpl(const double* update) {
    Eigen::Map<const Vec3_t> v(update);
    _estimate += v;
}

#define USE_PADDED_CAMERA_INTRINSICS_VERTEX

#ifdef USE_PADDED_CAMERA_INTRINSICS_VERTEX
// temp
using camera_intrinsics_vertex = camera_intrinsics_vertex_2;
using camera_intrinsics_vertex_type = Vec3_t;



//#include "landmark_vertex.h"
//using camera_intrinsics_vertex = landmark_vertex;
//using camera_intrinsics_vertex_type = Vec3_t;

#else
// temp
using camera_intrinsics_vertex = camera_intrinsics_vertex_1;

#endif

} // namespace internal
} // namespace optimize
} // namespace stella_vslam

