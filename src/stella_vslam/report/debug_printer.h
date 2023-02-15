/** \file
 * \brief Some useful functions for printing keyframes, landmarks etc.
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include "stella_vslam/exports.h"
#include <Eigen/Core>
#include <spdlog/spdlog.h>

namespace stella_vslam_bfx
{
class STELLA_VSLAM_API debug_printer {
public:
    static int precision;
    static Eigen::IOFormat eigen_format;

    template<typename T>
    static void print_keyframes(const T& keyframes, spdlog::level::level_enum log_level = spdlog::level::trace);

    template<typename T>
    static void print_landmarks(const T& landmarks, spdlog::level::level_enum log_level = spdlog::level::trace);

};

} // namespace stella_vslam_bfx
