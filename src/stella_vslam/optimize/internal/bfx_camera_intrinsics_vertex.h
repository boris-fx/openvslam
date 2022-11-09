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
 * Used landmark_vertex as a base
 */
class bfx_camera_intrinsics_vertex final : public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bfx_camera_intrinsics_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update) override;
};

inline bfx_camera_intrinsics_vertex::bfx_camera_intrinsics_vertex()
    : g2o::BaseVertex<1, double>() {}

inline bool bfx_camera_intrinsics_vertex::read(std::istream& is) {
    Vec3_t lv;
    is >> _estimate;
    return true;
}

inline bool bfx_camera_intrinsics_vertex::write(std::ostream& os) const {
    const double focalLengthXPixels = estimate();
    os << focalLengthXPixels << " ";
    return os.good();
}

inline void bfx_camera_intrinsics_vertex::setToOriginImpl() {
    _estimate = 1000.0; // Not sure what this is used for
}

inline void bfx_camera_intrinsics_vertex::oplusImpl(const double* update) {
    _estimate += *update;
}

} // namespace internal
} // namespace optimize
} // namespace stella_vslam

