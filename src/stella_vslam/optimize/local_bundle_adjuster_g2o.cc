#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/data/map_camera_helpers.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/optimize/local_bundle_adjuster_g2o.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"
#include "stella_vslam/optimize/terminate_action.h"
#include "stella_vslam/optimize/internal/landmark_vertex_container.h"
#include "stella_vslam/optimize/internal/marker_vertex_container.h"
#include "stella_vslam/optimize/internal/se3/shot_vertex_container.h"
#include "stella_vslam/optimize/internal/se3/reproj_edge_wrapper.h"
#include "stella_vslam/util/converter.h"
#include "stella_vslam/report/metrics.h"

#include <unordered_map>

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace optimize {

local_bundle_adjuster_g2o::local_bundle_adjuster_g2o(const stella_vslam_bfx::config_settings& settings,
                                             const unsigned int num_first_iter,
                                             const unsigned int num_second_iter)
    : num_first_iter_(num_first_iter), num_second_iter_(num_second_iter),
      use_additional_keyframes_for_monocular_(settings.use_additional_keyframes_for_monocular_) {}

std::string timestamp_as_string(double timestamp)
{
    // Print as ints (assuming timestamps are separated by 1.0)
    return std::to_string(int(timestamp + 0.5));
}

std::string keyframe_map_as_string(std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> const& keyfrms)
{
    if (keyfrms.empty())
        return std::string();
    // Collect timestamps
    std::vector<double> timestamps;
    for (auto const& key : keyfrms)
        timestamps.push_back(key.second->timestamp_);
    // Print as ints (assuming timestamps are separated by 1.0)
    std::stringstream ss;
    for (int i = 0; i < timestamps.size() - 1; ++i)
        ss << timestamp_as_string(timestamps[i]) << ", ";
    ss << timestamp_as_string(*timestamps.rbegin());
    return ss.str();
}

void print_keyframe_info(std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> const& fixed_keyfrms,
                         std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> const& local_keyfrms,
                         const std::shared_ptr<stella_vslam::data::keyframe>& curr_keyfrm, int map_frame_count)
{
    std::stringstream ss;

    // Print a more detailed list showing which keyframes are local and fixed 
    //ss << "local_bundle_adjuster frame: " << (curr_keyfrm ? timestamp_as_string(curr_keyfrm->timestamp_) : std::string("{no current frame}"))
    //                                      << " - local frames: " << keyframe_map_as_string(local_keyfrms)
    //                                      << " - fixed frames : " << keyframe_map_as_string(fixed_keyfrms) 
    //                                      << " - keyframe count: " << map_frame_count << std::endl;

    ss << "local_bundle_adjuster frame: " << (curr_keyfrm ? timestamp_as_string(curr_keyfrm->timestamp_) : std::string("{no current frame}"))
        << " - local frames: " << local_keyfrms.size()
        << " - fixed frames : " << fixed_keyfrms.size()
        << " - keyframe count: " << map_frame_count << std::endl;

    spdlog::info("{}", ss.str());
}

bool bundle_adjust_map(data::map_database* map_db)
{
    global_bundle_adjuster optimizer(100, true); // iterations, use_huber

    auto keys = map_db->get_all_keyframes();

    stella_vslam::camera::base* camera = stella_vslam_bfx::camera_from_keyframes(keys);
    double focal_before = stella_vslam_bfx::focal_length_x_pixels_from_camera(camera);


    std::unordered_set<unsigned int> optimized_keyfrm_ids;
    std::unordered_set<unsigned int> optimized_landmark_ids;
    eigen_alloc_unord_map<unsigned int, Vec3_t> lm_to_pos_w_after_global_BA;
    eigen_alloc_unord_map<unsigned int, Mat44_t> keyfrm_to_pose_cw_after_global_BA;
    bool* const force_stop_flag(nullptr);
    int num_iter(100);
    bool general_bundle(true);
    bool camera_was_modified;
    bool ok = optimizer.optimize(keys, optimized_keyfrm_ids, optimized_landmark_ids,
                                    lm_to_pos_w_after_global_BA, keyfrm_to_pose_cw_after_global_BA,
                                    force_stop_flag, num_iter, general_bundle, &camera_was_modified);


    double focal_after = stella_vslam_bfx::focal_length_x_pixels_from_camera(camera);
    spdlog::warn("_bundla_adjust_map_ Focal length {:03.2f} -> {:03.2f} ->", focal_before, focal_after);


    return ok;;
}

/** This function allocates a camera vertex with \c new if necessary (ownership will pass to the optimiser it's added to) **/
internal::camera_intrinsics_vertex* create_camera_intrinsics_vertex(const std::shared_ptr<unsigned int> offset,
    stella_vslam::camera::base const* camera)
{
    if (!camera || !camera->autocalibration_parameters_.optimise_focal_length)
        return nullptr;

    // Convert the camera intrinsics to a g2o vertex
    auto vtx = new internal::camera_intrinsics_vertex();

    const auto vtx_id = *offset;
    (*offset)++;

    vtx->setId(vtx_id);
#ifdef USE_PADDED_CAMERA_INTRINSICS_VERTEX
    vtx->setEstimate(internal::camera_intrinsics_vertex_type(stella_vslam_bfx::focal_length_x_pixels_from_camera(camera), 0.0, 0.0));
#else
    vtx->setEstimate(*autocalibration_wrapper.fx);
#endif
    vtx->setFixed(false);
    vtx->setMarginalized(false); // "this node is marginalized out during the optimization"
                                 // This is set to false for camera positions, true for points

    return vtx;
}

void local_bundle_adjuster_g2o::optimize(data::map_database* map_db,
                                         const std::shared_ptr<stella_vslam::data::keyframe>& curr_keyfrm, bool* const force_stop_flag) const {

    //optimize2(map_db, curr_keyfrm, force_stop_flag);
    //return;

    // 1. Aggregate the local and fixed keyframes, and local landmarks

    //spdlog::info("local_bundle_adjuster start {}", (curr_keyfrm ? curr_keyfrm->timestamp_ : -1));

    // Correct the local keyframes of the current keyframe
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> local_keyfrms;
    bool has_scale = false;

    local_keyfrms[curr_keyfrm->id_] = curr_keyfrm;
    const auto curr_covisibilities = curr_keyfrm->graph_node_->get_covisibilities();
    for (const auto& local_keyfrm : curr_covisibilities) {
        if (!local_keyfrm) {
            continue;
        }
        if (local_keyfrm->will_be_erased()) {
            continue;
        }
        if (local_keyfrm->graph_node_->is_spanning_root()) {
            continue;
        }

        local_keyfrms[local_keyfrm->id_] = local_keyfrm;
        if (local_keyfrm->camera_->setup_type_ != camera::setup_type_t::Monocular) {
            has_scale = true;
        }
    }

    // Correct landmarks seen in local keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::landmark>> local_lms;

    for (const auto& local_keyfrm : local_keyfrms) {
        const auto landmarks = local_keyfrm.second->get_landmarks();
        for (const auto& local_lm : landmarks) {
            if (!local_lm) {
                continue;
            }
            if (local_lm->will_be_erased()) {
                continue;
            }

            // Avoid duplication
            if (local_lms.count(local_lm->id_)) {
                continue;
            }

            local_lms[local_lm->id_] = local_lm;
        }
    }

    // Correct markers seen in local keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::marker>> local_mkrs;

    for (auto local_keyfrm : local_keyfrms) {
        const auto markers = local_keyfrm.second->get_markers();
        for (auto local_mkr : markers) {
            if (!local_mkr) {
                continue;
            }

            // Avoid duplication
            if (local_mkrs.count(local_mkr->id_)) {
                continue;
            }

            local_mkrs[local_mkr->id_] = local_mkr;
        }
    }

    // Fixed keyframes: keyframes which observe local landmarks but which are NOT in local keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> fixed_keyfrms;

    for (const auto& local_lm : local_lms) {
        const auto observations = local_lm.second->get_observations();
        for (const auto& obs : observations) {
            const auto fixed_keyfrm = obs.first.lock();
            if (!fixed_keyfrm) {
                continue;
            }
            if (fixed_keyfrm->will_be_erased()) {
                continue;
            }

            // Do not add if it's in the local keyframes
            if (local_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            // Avoid duplication
            if (fixed_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            fixed_keyfrms[fixed_keyfrm->id_] = fixed_keyfrm;
        }
    }

    if (use_additional_keyframes_for_monocular_) {
        // Ensure that there are always at least two fixed keyframes
        auto additional_keyfrms_size = 2 - fixed_keyfrms.size();
        if (!has_scale && fixed_keyfrms.size() < 2 && local_keyfrms.size() > additional_keyfrms_size) {
            for (unsigned int i = 0; i < additional_keyfrms_size; ++i) {
                auto itr = local_keyfrms.begin();
                auto keyfrm_id = itr->first;
                auto keyfrm = itr->second;
                local_keyfrms.erase(keyfrm_id);
                fixed_keyfrms[keyfrm_id] = keyfrm;
            }
        }
    }

    stella_vslam::camera::base *camera = stella_vslam_bfx::camera_from_keyframes(local_keyfrms);

    // 2. Construct an optimizer

    g2o::OptimizationAlgorithmLevenberg* algorithm(nullptr);
    // Block solver takes ownership of linear_solver
    // algorithm takes ownership of block_solver
    if (camera->autocalibration_parameters_.optimise_focal_length) {
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    }
    else {
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    }

    g2o::SparseOptimizer optimizer;
    auto terminateAction = new terminate_action;
    terminateAction->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);
    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // 3. Convert each of the keyframe to the g2o vertex, then set it to the optimizer

    // Container of the shot vertices
    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, local_keyfrms.size() + fixed_keyfrms.size());
    // Save the converted keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> all_keyfrms;

    // Set the local keyframes to the optimizer
    for (const auto& id_local_keyfrm_pair : local_keyfrms) {
        const auto& local_keyfrm = id_local_keyfrm_pair.second;

        all_keyfrms.emplace(id_local_keyfrm_pair);
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(local_keyfrm, false);
        optimizer.addVertex(keyfrm_vtx);
    }

    // Set the fixed keyframes to the optimizer
    for (const auto& id_fixed_keyfrm_pair : fixed_keyfrms) {
        const auto& fixed_keyfrm = id_fixed_keyfrm_pair.second;

        all_keyfrms.emplace(id_fixed_keyfrm_pair);
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(fixed_keyfrm, true);
        optimizer.addVertex(keyfrm_vtx);
    }

    print_keyframe_info(fixed_keyfrms, local_keyfrms, curr_keyfrm, map_db->get_num_keyframes());

    // 3(b). Camera intrinsics
    internal::camera_intrinsics_vertex* camera_intrinsics_vtx = create_camera_intrinsics_vertex(vtx_id_offset, camera);
    if (camera_intrinsics_vtx)
        optimizer.addVertex(camera_intrinsics_vtx);

    // 4. Connect the vertices of the keyframe and the landmark by using an edge of reprojection constraint

    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, local_lms.size());

    // Container of the reprojection edges
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(all_keyfrms.size() * local_lms.size());

    // Chi-squared value with significance level of 5%
    // Two degree-of-freedom (n=2)
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Three degree-of-freedom (n=3)
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (const auto& id_local_lm_pair : local_lms) {
        const auto local_lm = id_local_lm_pair.second;
        const auto observations = local_lm->get_observations();
        if (observations.empty()) {
            spdlog::warn("empty observation");
            continue;
        }

        // Convert the landmark to the g2o vertex, then set to the optimizer
        auto lm_vtx = lm_vtx_container.create_vertex(local_lm, false);
        optimizer.addVertex(lm_vtx);

        for (const auto& obs : observations) {
            const auto keyfrm = obs.first.lock();
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }
            if (!keyfrm_vtx_container.contain(keyfrm)) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->frm_obs_.undist_keypts_.at(idx);
            const float x_right = keyfrm->frm_obs_.stereo_x_right_.empty() ? -1.0f : keyfrm->frm_obs_.stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->orb_params_->inv_level_sigma_sq_.at(undist_keypt.octave);
            const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, local_lm, lm_vtx, camera_intrinsics_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
        }
    }

    // Container of the reprojection edges for corners of markers
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, local_mkrs.size());
    std::vector<reproj_edge_wrapper> mkr_reproj_edge_wraps;
    mkr_reproj_edge_wraps.reserve(all_keyfrms.size() * local_mkrs.size());

    for (auto& id_local_mkr_pair : local_mkrs) {
        auto mkr = id_local_mkr_pair.second;
        if (!mkr) {
            continue;
        }

        // Convert the corners to the g2o vertex, then set it to the optimizer
        auto corner_vertices = marker_vtx_container.create_vertices(mkr, true);
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            const auto corner_vtx = corner_vertices[corner_idx];
            optimizer.addVertex(corner_vtx);

            for (const auto& keyfrm : mkr->observations_) {
                if (!keyfrm) {
                    continue;
                }
                if (keyfrm->will_be_erased()) {
                    continue;
                }
                if (!keyfrm_vtx_container.contain(keyfrm)) {
                    continue;
                }
                const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
                const auto& mkr_2d = keyfrm->markers_2d_.at(mkr->id_);
                const auto& undist_pt = mkr_2d.undist_corners_.at(corner_idx);
                const float x_right = -1.0;
                const float inv_sigma_sq = 1.0;
                auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, nullptr, corner_vtx, camera_intrinsics_vtx,
                                                            0, undist_pt.x, undist_pt.y, x_right,
                                                            inv_sigma_sq, 0.0, false);
                mkr_reproj_edge_wraps.push_back(reproj_edge_wrap);
                optimizer.addEdge(reproj_edge_wrap.edge_);
            }
        }
    }

    // 5. Perform the first optimization

    if (force_stop_flag && *force_stop_flag) {
        return;
    }


    if (camera_intrinsics_vtx) {
        double focal_length_x_pixels = camera_intrinsics_vtx->focal_length_x_pixels();
        spdlog::warn("             before first -> {:03.2f} ->", focal_length_x_pixels);
    }


    optimizer.initializeOptimization();
    optimizer.optimize(num_first_iter_);


    if (camera_intrinsics_vtx) {
        double focal_length_x_pixels = camera_intrinsics_vtx->focal_length_x_pixels();
        spdlog::warn("             after first -> {:03.2f} ->", focal_length_x_pixels);
    }

    // 6. Discard outliers, then perform the second optimization

    bool run_robust_BA = true;

    if (force_stop_flag && *force_stop_flag) {
        run_robust_BA = false;
    }

    if (run_robust_BA) {
        for (auto& reproj_edge_wrap : reproj_edge_wraps) {
            auto edge = reproj_edge_wrap.edge_;

            const auto& local_lm = reproj_edge_wrap.lm_;
            if (local_lm->will_be_erased()) {
                continue;
            }

            if (reproj_edge_wrap.is_monocular_) {
                if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }
            else {
                if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }

            edge->setRobustKernel(nullptr);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(num_second_iter_);
    }

    delete terminateAction;

    // 7. Count the outliers

    std::vector<std::pair<std::shared_ptr<data::keyframe>, std::shared_ptr<data::landmark>>> outlier_observations;
    outlier_observations.reserve(reproj_edge_wraps.size());

    for (auto& reproj_edge_wrap : reproj_edge_wraps) {
        auto edge = reproj_edge_wrap.edge_;

        const auto& local_lm = reproj_edge_wrap.lm_;
        if (local_lm->will_be_erased()) {
            continue;
        }

        if (reproj_edge_wrap.is_monocular_) {
            if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
        else {
            if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
    }

    // 8. Update the information

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (const auto& outlier_obs : outlier_observations) {
            const auto& keyfrm = outlier_obs.first;
            const auto& lm = outlier_obs.second;
            keyfrm->erase_landmark(lm);
            lm->erase_observation(map_db, keyfrm);
            if (!lm->will_be_erased()) {
                lm->compute_descriptor();
                lm->update_mean_normal_and_obs_scale_variance();
            }
        }

        if (camera_intrinsics_vtx) {
            double focal_length_x_pixels = camera_intrinsics_vtx->focal_length_x_pixels();
            stella_vslam_bfx::set_camera_focal_length_x_pixels(camera, focal_length_x_pixels);
            auto stage = stella_vslam_bfx::focal_estimation_stage::local_optimisation;
            stella_vslam_bfx::metrics::get_instance()->submit_intermediate_focal_estimate(stage, focal_length_x_pixels);
            spdlog::warn("local bundle focal length -> {:03.2f}", focal_length_x_pixels);
        }

        for (const auto& id_local_keyfrm_pair : local_keyfrms) {
            const auto& local_keyfrm = id_local_keyfrm_pair.second;

            auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(local_keyfrm);
            local_keyfrm->set_pose_cw(keyfrm_vtx->estimate());

            if (camera_intrinsics_vtx) { // NB: camera has already been updated with the new focal length
                local_keyfrm->frm_obs_.bearings_.clear();
                camera->convert_keypoints_to_bearings(local_keyfrm->frm_obs_.undist_keypts_, local_keyfrm->frm_obs_.bearings_);
            }
        }

        for (const auto& id_local_lm_pair : local_lms) {
            const auto& local_lm = id_local_lm_pair.second;
            if (local_lm->will_be_erased()) {
                continue;
            }

            auto lm_vtx = lm_vtx_container.get_vertex(local_lm);
            local_lm->set_pos_in_world(lm_vtx->estimate());
            local_lm->update_mean_normal_and_obs_scale_variance();
        }

    }
}

void local_bundle_adjuster_g2o::optimize2(data::map_database* map_db,
    const std::shared_ptr<stella_vslam::data::keyframe>& curr_keyfrm, bool* const force_stop_flag) const {
    // 1. Aggregate the local and fixed keyframes, and local landmarks

    //spdlog::info("local_bundle_adjuster start {}", (curr_keyfrm ? curr_keyfrm->timestamp_ : -1));

    // Correct the local keyframes of the current keyframe
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> local_keyfrms;
    bool has_scale = false;

    local_keyfrms[curr_keyfrm->id_] = curr_keyfrm;
    const auto curr_covisibilities = curr_keyfrm->graph_node_->get_covisibilities();
    for (const auto& local_keyfrm : curr_covisibilities) {
        if (!local_keyfrm) {
            continue;
        }
        if (local_keyfrm->will_be_erased()) {
            continue;
        }
        if (local_keyfrm->graph_node_->is_spanning_root()) {
            continue;
        }

        local_keyfrms[local_keyfrm->id_] = local_keyfrm;
        if (local_keyfrm->camera_->setup_type_ != camera::setup_type_t::Monocular) {
            has_scale = true;
        }
    }

    // Correct landmarks seen in local keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::landmark>> local_lms;

    for (const auto& local_keyfrm : local_keyfrms) {
        const auto landmarks = local_keyfrm.second->get_landmarks();
        for (const auto& local_lm : landmarks) {
            if (!local_lm) {
                continue;
            }
            if (local_lm->will_be_erased()) {
                continue;
            }

            // Avoid duplication
            if (local_lms.count(local_lm->id_)) {
                continue;
            }

            local_lms[local_lm->id_] = local_lm;
        }
    }

    // Correct markers seen in local keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::marker>> local_mkrs;

    for (auto local_keyfrm : local_keyfrms) {
        const auto markers = local_keyfrm.second->get_markers();
        for (auto local_mkr : markers) {
            if (!local_mkr) {
                continue;
            }

            // Avoid duplication
            if (local_mkrs.count(local_mkr->id_)) {
                continue;
            }

            local_mkrs[local_mkr->id_] = local_mkr;
        }
    }

    // Fixed keyframes: keyframes which observe local landmarks but which are NOT in local keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> fixed_keyfrms;

    for (const auto& local_lm : local_lms) {
        const auto observations = local_lm.second->get_observations();
        for (const auto& obs : observations) {
            const auto fixed_keyfrm = obs.first.lock();
            if (!fixed_keyfrm) {
                continue;
            }
            if (fixed_keyfrm->will_be_erased()) {
                continue;
            }

            // Do not add if it's in the local keyframes
            if (local_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            // Avoid duplication
            if (fixed_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }

            fixed_keyfrms[fixed_keyfrm->id_] = fixed_keyfrm;
        }
    }

    if (use_additional_keyframes_for_monocular_) {
        // Ensure that there are always at least two fixed keyframes
        auto additional_keyfrms_size = 2 - fixed_keyfrms.size();
        if (!has_scale && fixed_keyfrms.size() < 2 && local_keyfrms.size() > additional_keyfrms_size) {
            for (unsigned int i = 0; i < additional_keyfrms_size; ++i) {
                auto itr = local_keyfrms.begin();
                auto keyfrm_id = itr->first;
                auto keyfrm = itr->second;
                local_keyfrms.erase(keyfrm_id);
                fixed_keyfrms[keyfrm_id] = keyfrm;
            }
        }
    }


    // TEST
    local_keyfrms.merge(fixed_keyfrms);

    stella_vslam::camera::base* camera = stella_vslam_bfx::camera_from_keyframes(local_keyfrms);

    // 2. Construct an optimizer

    g2o::OptimizationAlgorithmLevenberg* algorithm(nullptr);
    // Block solver takes ownership of linear_solver
    // algorithm takes ownership of block_solver
    if (camera->autocalibration_parameters_.optimise_focal_length) {
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    }
    else {
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    }

    g2o::SparseOptimizer optimizer;
    auto terminateAction = new terminate_action;
    //terminateAction->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);
    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // 3. Convert each of the keyframe to the g2o vertex, then set it to the optimizer

    // Container of the shot vertices
    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, local_keyfrms.size() + fixed_keyfrms.size());
    // Save the converted keyframes
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> all_keyfrms;

    // Set the local keyframes to the optimizer
    for (const auto& id_local_keyfrm_pair : local_keyfrms) {
        const auto& local_keyfrm = id_local_keyfrm_pair.second;

        all_keyfrms.emplace(id_local_keyfrm_pair);
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(local_keyfrm, false);
        optimizer.addVertex(keyfrm_vtx);
    }

    // Set the fixed keyframes to the optimizer
    for (const auto& id_fixed_keyfrm_pair : fixed_keyfrms) {
        const auto& fixed_keyfrm = id_fixed_keyfrm_pair.second;

        all_keyfrms.emplace(id_fixed_keyfrm_pair);
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(fixed_keyfrm, true);
        optimizer.addVertex(keyfrm_vtx);
    }

    print_keyframe_info(fixed_keyfrms, local_keyfrms, curr_keyfrm, map_db->get_num_keyframes());

    // 3(b). Camera intrinsics
    internal::camera_intrinsics_vertex* camera_intrinsics_vtx = create_camera_intrinsics_vertex(vtx_id_offset, camera);

    // TEST
    //double extra_focal_pixels(5.0);
    //std::shared_ptr<stella_vslam::camera::base> camera_plus_extra_focal;
    //if (camera->model_type_ == camera::model_type_t::Perspective) {
    //    auto c = static_cast<camera::perspective const*>(camera);
    //    camera_plus_extra_focal = std::make_shared<camera::perspective>(*c);
    //    stella_vslam_bfx::set_camera_focal_length_x_pixels(camera_plus_extra_focal.get(), extra_focal_pixels + stella_vslam_bfx::focal_length_x_pixels_from_camera(camera));
    //}
    //internal::camera_intrinsics_vertex* camera_intrinsics_vtx = create_camera_intrinsics_vertex(vtx_id_offset, camera_plus_extra_focal.get());

    if (camera_intrinsics_vtx)
        optimizer.addVertex(camera_intrinsics_vtx);

    // 4. Connect the vertices of the keyframe and the landmark by using an edge of reprojection constraint

    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, local_lms.size());

    // Container of the reprojection edges
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(all_keyfrms.size() * local_lms.size());

    // Chi-squared value with significance level of 5%
    // Two degree-of-freedom (n=2)
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Three degree-of-freedom (n=3)
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (const auto& id_local_lm_pair : local_lms) {
        const auto local_lm = id_local_lm_pair.second;
        const auto observations = local_lm->get_observations();
        if (observations.empty()) {
            spdlog::warn("empty observation");
            continue;
        }

        // Convert the landmark to the g2o vertex, then set to the optimizer
        auto lm_vtx = lm_vtx_container.create_vertex(local_lm, false);
        optimizer.addVertex(lm_vtx);

        for (const auto& obs : observations) {
            const auto keyfrm = obs.first.lock();
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }
            if (!keyfrm_vtx_container.contain(keyfrm)) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->frm_obs_.undist_keypts_.at(idx);
            const float x_right = keyfrm->frm_obs_.stereo_x_right_.empty() ? -1.0f : keyfrm->frm_obs_.stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->orb_params_->inv_level_sigma_sq_.at(undist_keypt.octave);
            const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                ? sqrt_chi_sq_2D
                : sqrt_chi_sq_3D;
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, local_lm, lm_vtx, camera_intrinsics_vtx,
                idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                inv_sigma_sq, sqrt_chi_sq);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
        }
    }

    // Container of the reprojection edges for corners of markers
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, local_mkrs.size());
    std::vector<reproj_edge_wrapper> mkr_reproj_edge_wraps;
    mkr_reproj_edge_wraps.reserve(all_keyfrms.size() * local_mkrs.size());

    for (auto& id_local_mkr_pair : local_mkrs) {
        auto mkr = id_local_mkr_pair.second;
        if (!mkr) {
            continue;
        }

        // Convert the corners to the g2o vertex, then set it to the optimizer
        auto corner_vertices = marker_vtx_container.create_vertices(mkr, true);
        for (unsigned int corner_idx = 0; corner_idx < corner_vertices.size(); ++corner_idx) {
            const auto corner_vtx = corner_vertices[corner_idx];
            optimizer.addVertex(corner_vtx);

            for (const auto& keyfrm : mkr->observations_) {
                if (!keyfrm) {
                    continue;
                }
                if (keyfrm->will_be_erased()) {
                    continue;
                }
                if (!keyfrm_vtx_container.contain(keyfrm)) {
                    continue;
                }
                const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
                const auto& mkr_2d = keyfrm->markers_2d_.at(mkr->id_);
                const auto& undist_pt = mkr_2d.undist_corners_.at(corner_idx);
                const float x_right = -1.0;
                const float inv_sigma_sq = 1.0;
                auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, nullptr, corner_vtx, camera_intrinsics_vtx,
                    0, undist_pt.x, undist_pt.y, x_right,
                    inv_sigma_sq, 0.0, false);
                mkr_reproj_edge_wraps.push_back(reproj_edge_wrap);
                optimizer.addEdge(reproj_edge_wrap.edge_);
            }
        }
    }

    // 5. Perform the first optimization

    if (force_stop_flag && *force_stop_flag) {
        return;
    }


    if (camera_intrinsics_vtx) {
        double focal_length_x_pixels = camera_intrinsics_vtx->focal_length_x_pixels();
        spdlog::warn("             before first -> {:03.2f} ->", focal_length_x_pixels);
    }


    optimizer.initializeOptimization();
    optimizer.optimize(100);


    if (camera_intrinsics_vtx) {
        double focal_length_x_pixels = camera_intrinsics_vtx->focal_length_x_pixels();
        spdlog::warn("             after first -> {:03.2f} iter {}->", focal_length_x_pixels, num_first_iter_);
    }

    // 6. Discard outliers, then perform the second optimization

    bool run_robust_BA = true;

    if (force_stop_flag && *force_stop_flag) {
        run_robust_BA = false;
    }

    if (run_robust_BA) {
        for (auto& reproj_edge_wrap : reproj_edge_wraps) {
            auto edge = reproj_edge_wrap.edge_;

            const auto& local_lm = reproj_edge_wrap.lm_;
            if (local_lm->will_be_erased()) {
                continue;
            }

            if (reproj_edge_wrap.is_monocular_) {
                if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }
            else {
                if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                    reproj_edge_wrap.set_as_outlier();
                }
            }

            edge->setRobustKernel(nullptr);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(num_second_iter_);
    }

    //delete terminateAction;

    // 7. Count the outliers

    std::vector<std::pair<std::shared_ptr<data::keyframe>, std::shared_ptr<data::landmark>>> outlier_observations;
    outlier_observations.reserve(reproj_edge_wraps.size());

    for (auto& reproj_edge_wrap : reproj_edge_wraps) {
        auto edge = reproj_edge_wrap.edge_;

        const auto& local_lm = reproj_edge_wrap.lm_;
        if (local_lm->will_be_erased()) {
            continue;
        }

        if (reproj_edge_wrap.is_monocular_) {
            if (chi_sq_2D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
        else {
            if (chi_sq_3D < edge->chi2() || !reproj_edge_wrap.depth_is_positive()) {
                outlier_observations.emplace_back(std::make_pair(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_));
            }
        }
    }

    // 8. Update the information

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (const auto& outlier_obs : outlier_observations) {
            const auto& keyfrm = outlier_obs.first;
            const auto& lm = outlier_obs.second;
            keyfrm->erase_landmark(lm);
            lm->erase_observation(map_db, keyfrm);
            if (!lm->will_be_erased()) {
                lm->compute_descriptor();
                lm->update_mean_normal_and_obs_scale_variance();
            }
        }

        if (camera_intrinsics_vtx) {
            double focal_length_x_pixels = camera_intrinsics_vtx->focal_length_x_pixels();
            stella_vslam_bfx::set_camera_focal_length_x_pixels(camera, focal_length_x_pixels);
            auto stage = stella_vslam_bfx::focal_estimation_stage::local_optimisation;
            stella_vslam_bfx::metrics::get_instance()->submit_intermediate_focal_estimate(stage, focal_length_x_pixels);
            spdlog::warn("local bundle focal length -> {:03.2f}", focal_length_x_pixels);
        }

        for (const auto& id_local_keyfrm_pair : local_keyfrms) {
            const auto& local_keyfrm = id_local_keyfrm_pair.second;

            auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(local_keyfrm);
            local_keyfrm->set_pose_cw(keyfrm_vtx->estimate());

            if (camera_intrinsics_vtx) { // NB: camera has already been updated with the new focal length
                local_keyfrm->frm_obs_.bearings_.clear();
                camera->convert_keypoints_to_bearings(local_keyfrm->frm_obs_.undist_keypts_, local_keyfrm->frm_obs_.bearings_);
            }
        }

        for (const auto& id_local_lm_pair : local_lms) {
            const auto& local_lm = id_local_lm_pair.second;
            if (local_lm->will_be_erased()) {
                continue;
            }

            auto lm_vtx = lm_vtx_container.get_vertex(local_lm);
            local_lm->set_pos_in_world(lm_vtx->estimate());
            local_lm->update_mean_normal_and_obs_scale_variance();
        }

    }

    //static int hit_count(0);
    //++hit_count;
    //spdlog::warn("hit_count a {}", hit_count);
    //spdlog::info("hit_count b {}", hit_count);
    //if (hit_count == 10)
    //    bool ok_global = bundle_adjust_map(map_db);

}

} // namespace optimize
} // namespace stella_vslam
