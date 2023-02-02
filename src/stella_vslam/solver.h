/** \file
 * \brief Definition of the interface between the stella-vslam based solver and apps using it (Mocha, run_video_slam, etc...)
 *
 * Copyright (c) 2022 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <map>
#include <functional>

#include <Eigen/Geometry>

#include "stella_vslam/exports.h"
#include <stella_vslam/camera/perspective.h>

namespace cv {
    class Mat;
}

namespace stella_vslam {
    class system;
    class config;
}

namespace stella_vslam_bfx {

/**
 * \brief A camera solve (a set of 3D points, and a set of per=frame camera positions)
 */
class solve {
public:
    std::map<int, Eigen::Matrix4d> frame_to_camera;
    std::vector<Eigen::Vector3d> world_points;
    std::shared_ptr<stella_vslam::camera::perspective> camera_lens;
};

/**
 * \brief Frame-level results pushed from the solver for possible user display
 */
class frame_display_data {
    // Some data that Mocha can use to display interesting things on the screen during tracking,
    // e.g. a set of 3D points and a camera, or a set or 2D image features
};

/**
 * \brief Main interface object for the stella-vslam based camera solver
 *
 */
class STELLA_VSLAM_API solver {
public:
    solver(const std::shared_ptr<stella_vslam::config>& cfg,
           const std::string& vocab_file_path,
           std::function<bool(int, cv::Mat&)> get_frame);
    virtual ~solver();

    /// Optional callback
    void set_progress_callback(std::function<void(float)> set_progress);
    void set_stage_description_callback(std::function<void(std::string)> set_stage_description);
    void set_display_frame_callback(std::function<void(std::shared_ptr<frame_display_data>)> display_frame);
    void set_cancel_callback(std::function<bool()> cancel);

    enum tracking_direction {
        tracking_direction_forwards,
        tracking_direction_backwards
    };

    /// Track (or retrack)
    bool track_frame_range(int begin, int end, tracking_direction direction, solve* final_solve);

public:
    /// Should be protected, but this is used by pangolin_viewer
    std::shared_ptr<stella_vslam::system> system();

protected:
    std::function<bool(int, cv::Mat&)> get_frame_;

    std::function<void(float)> set_progress_;
    std::function<void(std::string)> set_stage_description_;
    std::function<void(std::shared_ptr<frame_display_data>)> display_frame_;
    std::function<bool()> cancel_;

    std::shared_ptr<stella_vslam::system> slam_;
    std::shared_ptr<stella_vslam::config> cfg_;
};

} // namespace stella_vslam_bfx
