/** \file
 * \brief Definition of fundamental matrix consistency enforcer  
 *
 * Copyright (c) 2023 Boris Fx Inc. All rights reserved
 */
#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>
#include <list>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/types.hpp>

#include "stella_vslam/exports.h"

namespace stella_vslam::camera { class base; }

namespace stella_vslam_bfx {

struct frame_pair_matches
{
    using frame_id = double; // double because stella only knows about timestamps (double)
    frame_id frame_id_1;
    frame_id frame_id_2;
    std::vector<std::pair<int, int>> unguided_matches_12; // All unguided matches
    std::vector<bool> guided_inlier_matches; // Input inliers to the f-matrix
    Eigen::Matrix3d F_21; // fundamental matrix calculated during guided matching
};

struct input_matches {
    std::map<frame_pair_matches::frame_id, std::vector<cv::KeyPoint>> undist_keypts; // frame_id to keypoints
    std::list<frame_pair_matches> frame_matches;
};

class STELLA_VSLAM_API focal_length_estimator
{
public:

    focal_length_estimator& operator=(const focal_length_estimator&) = delete;


    static focal_length_estimator* get_instance();

    static void clear(); // clear for a new camera track

    std::array<double, 2> current_frame_pair; // Set the current frame identifiers - typically timestamps which aren't known when add_frame_pair() is called

    // Always set current_frame_pair before calling this
    bool add_frame_pair(const std::vector<cv::KeyPoint>& undist_keypts_1,
                        const std::vector<cv::KeyPoint>& undist_keypts_2,
                        const std::vector<std::pair<int, int>>& input_matches_12,
                        std::vector<bool> const& input_inlier_matches,
                        const Eigen::Matrix3d F_21);

    bool run_optimisation(stella_vslam::camera::base* camera,
                          bool& focal_length_estimate_is_stable,
                          bool& focal_length_changed);

protected:

    inline static focal_length_estimator* instance{ nullptr };
    focal_length_estimator() = default;
    ~focal_length_estimator() = default;
    focal_length_estimator(const focal_length_estimator&) = default;

    input_matches m_matches;



public:

    static bool test(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
              const std::vector<std::pair<int, int>>& input_matches_12, std::vector<bool> const& input_inlier_matches,
              const Eigen::Matrix3d F_21, stella_vslam::camera::base* camera,
              bool &focal_length_estimate_is_stable, bool &focal_length_changed);

    struct error_graph_metrics
    {
        double error_for_max_focal_length;
        double focal_length_at_min_error;
        double de_df_plus;
        double de_df_minus;
        double error_min_value;
        double min_error_percent_max_focal_error;
        std::map<double, double> focal_length_to_cost;
        bool focal_length_from_f_only_is_stable;
        double focal_length_from_f_only;
    };

    static bool test(input_matches const& matches,
              stella_vslam::camera::base* camera,
              std::map<std::pair<int,int>, error_graph_metrics> *metrics);

    // calls bool test(input_matches const& matches)
    static bool test_v2(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
        const std::vector<std::pair<int, int>>& input_matches_12, std::vector<bool> const& input_inlier_matches,
        const Eigen::Matrix3d F_21, stella_vslam::camera::base* camera,
        bool& focal_length_estimate_is_stable, bool& focal_length_changed);

};


/**
* \brief Orthonormal representation of the fundamental matrix
*
* This representaion has 7-dof and can be used to generate any (3x3) fundamental matrix
* 
* A. Bartoli and P. Sturm. 'Nonlinear estimation of the fundamental matrix with minimal parameters'.
* IEEE Transactions on Pattern Analysis and Machine Intelligence, 26(3):426-432, March 2004.
*
**/
struct orthonormal_f
{
    Eigen::Matrix3d u;
    Eigen::Matrix3d v;
    double sigma;

    void from_fundamental_matrix(Eigen::Matrix3d const& f);

    void to_fundamental_matrix(Eigen::Matrix3d& f) const;

    void update(Eigen::Matrix<double, 7, 1> const& delta);

};

bool orthonormal_f_test(Eigen::Matrix3d const& f);

double naive_lerp(double a, double b, double t); // NB: c++20 has std::lerp, which can replace this 

// e.g. auto quartiles = quantile<double>(vec, { 0.25, 0.5, 0.75 });
template<typename T> std::vector<T> quantile(const std::vector<T>& data, const std::vector<T>& probs);

template<typename T> T median(const std::vector<T>& data);

} // namespace stella_vslam_bfx
