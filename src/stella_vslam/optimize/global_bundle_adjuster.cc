#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"
#include "stella_vslam/optimize/terminate_action.h"
#include "stella_vslam/optimize/internal/landmark_vertex_container.h"
#include "stella_vslam/optimize/internal/marker_vertex_container.h"
#include "stella_vslam/optimize/internal/se3/shot_vertex_container.h"
#include "stella_vslam/optimize/internal/se3/reproj_edge_wrapper.h"
#include "stella_vslam/util/converter.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#undef G2O_LINEAR_SOLVER_CLASS
#if defined(HAVE_G2O_SOLVER_CSPARSE)
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#define G2O_LINEAR_SOLVER_CLASS LinearSolverCSparse 
#else
#define G2O_LINEAR_SOLVER_CLASS LinearSolverEigen
#endif

#include <g2o/core/optimization_algorithm_levenberg.h>

namespace stella_vslam {
namespace optimize {

/** Populate the shared keyframe camera from a camera intrinsics vertex **/
struct focal_length_from_keyframes {
    double *fx = nullptr;
    double *fy = nullptr;
    stella_vslam_bfx::autocalibration_parameters* autocalibration_params = nullptr;

    focal_length_from_keyframes(std::vector<std::shared_ptr<data::keyframe>> const& keyfrms)
    {
        // Get the shared camera from a keyframe
        camera::base* camera(nullptr);
        for (const auto& keyfrm : keyfrms) {
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }
            camera = keyfrm->camera_;
            break;
        }

        if (!camera)
            return;

        autocalibration_params = &camera->autocalibration_parameters_;

        switch (camera->model_type_) {
            case camera::model_type_t::Perspective: {
                auto c = static_cast<camera::perspective*>(camera);
                fx = &c->fx_;
                fy = &c->fy_;
                break;
            }
            case camera::model_type_t::Fisheye: {
                auto c = static_cast<camera::fisheye*>(camera);
                fx = &c->fx_;
                fy = &c->fy_;
                break;
            }
            case camera::model_type_t::RadialDivision: {
                auto c = static_cast<camera::radial_division*>(camera);
                fx = &c->fx_;
                fy = &c->fy_;
                break;
            }
        }
    }
};
   
/** This function allocates a camera vertex with \c new if necessary (ownership will pass to the optimiser it's added to) **/
internal::bfx_camera_intrinsics_vertex* create_camera_intrinsics_vertex(const std::shared_ptr<unsigned int> offset,
                                                                        std::vector<std::shared_ptr<data::keyframe>> const& keyfrms)
{
    // We don't have access to the camera database, but we can get the shared camera from a keyframe
    camera::base* camera(nullptr);
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }
        camera = keyfrm->camera_;
        break;
    }

    if (!camera || !camera->autocalibration_parameters_.optimise_focal_length)
        return nullptr;

    double focal_length_x_pixels;
    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera);
            focal_length_x_pixels = c->fx_;
            break;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            focal_length_x_pixels = c->fx_;
            break;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division*>(camera);
            focal_length_x_pixels = c->fx_;
            break;
        }
        default:
            return nullptr;
    }

    // Convert the camera intrinsics to a g2o vertex
    auto vtx = new internal::bfx_camera_intrinsics_vertex();

    const auto vtx_id = *offset;
    (*offset)++;

    vtx->setId(vtx_id);
#ifdef USE_PADDED_CAMERA_INTRINSICS_VERTEX
    vtx->setEstimate(internal::bfx_camera_intrinsics_vertex_type(focal_length_x_pixels, 0.0, 0.0));
#else
    vtx->setEstimate(focal_length_x_pixels);
#endif
    vtx->setFixed(false);
    vtx->setMarginalized(false); // "this node is marginalized out during the optimization"
                                 // This is set to false for camera positions, true for points

    return vtx;
}

/** Populate the shared keyframe camera from a camera intrinsics vertex **/
bool populate_camera_from_vertex(std::vector<std::shared_ptr<data::keyframe>> const& keyfrms,
                                 internal::bfx_camera_intrinsics_vertex *vertex)
{
    if (!vertex)
        return false;

    // We don't have access to the camera database, but we can get the shared camera from a keyframe
    camera::base* camera(nullptr);
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }
        camera = keyfrm->camera_;
        break;
    }

    if (!camera || !camera->autocalibration_parameters_.optimise_focal_length)
        return false; // Nothing to do

#ifdef USE_PADDED_CAMERA_INTRINSICS_VERTEX
    double focal_length_x_pixels = vertex->estimate()(0);
#else
    double focal_length_x_pixels = vertex->estimate();
#endif

    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            camera::perspective* c = static_cast<camera::perspective*>(camera);
            double par = c->fy_ / c->fx_;
            c->fx_ = focal_length_x_pixels;
            c->fy_ = focal_length_x_pixels * par;
            return true;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            double par = c->fy_ / c->fx_;
            c->fx_ = focal_length_x_pixels;
            c->fy_ = focal_length_x_pixels * par;
            return true;
        }
        case camera::model_type_t::RadialDivision: {
            auto c = static_cast<camera::radial_division*>(camera);
            double par = c->fy_ / c->fx_;
            c->fx_ = focal_length_x_pixels;
            c->fy_ = focal_length_x_pixels * par;
            return true;
        }
    }
    return false;
}

void optimize_impl(g2o::SparseOptimizer& optimizer,
                   std::vector<std::shared_ptr<data::keyframe>>& keyfrms,
                   std::vector<std::shared_ptr<data::landmark>>& lms,
                   std::vector<std::shared_ptr<data::marker>>& markers,
                   std::vector<bool>& is_optimized_lm,
                   internal::se3::shot_vertex_container& keyfrm_vtx_container,
                   internal::landmark_vertex_container& lm_vtx_container,
                   internal::marker_vertex_container& marker_vtx_container,
                   internal::bfx_camera_intrinsics_vertex* camera_intrinsics_vtx,
                   unsigned int num_iter,
                   bool use_huber_kernel,
                   bool* const force_stop_flag) {
    // 2. Construct an optimizer

    g2o::OptimizationAlgorithmLevenberg* algorithm(nullptr);
    // Block solver takes ownership of linear_solver
    // algorithm takes ownership of block_solver
    if (camera_intrinsics_vtx) {
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::G2O_LINEAR_SOLVER_CLASS<g2o::BlockSolverX::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    }
    else {
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = g2o::make_unique<g2o::G2O_LINEAR_SOLVER_CLASS<g2o::BlockSolver_6_3::PoseMatrixType>>();
        block_solver = g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
        algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    }

    optimizer.setAlgorithm(algorithm);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // If there's a camera intrinsics g2o vertex, add it to the optimizer
    if (camera_intrinsics_vtx)
        optimizer.addVertex(camera_intrinsics_vtx);

    // 3. Convert each of the keyframe to the g2o vertex, then set it to the optimizer

    // Set the keyframes to the optimizer
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }

        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(keyfrm, keyfrm->id_ == 0);
        optimizer.addVertex(keyfrm_vtx);
    }

    // 4. Connect the vertices of the keyframe and the landmark by using reprojection edge

    // Container of the reprojection edges
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(10 * lms.size());

    // Chi-squared value with significance level of 5%
    // Two degree-of-freedom (n=2)
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // Three degree-of-freedom (n=3)
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (unsigned int i = 0; i < lms.size(); ++i) {
        const auto& lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // Convert the landmark to the g2o vertex, then set it to the optimizer
        auto lm_vtx = lm_vtx_container.create_vertex(lm, false);
        optimizer.addVertex(lm_vtx);

        unsigned int num_edges = 0;
        const auto observations = lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first.lock();
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
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, lm, lm_vtx, camera_intrinsics_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, use_huber_kernel);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
            ++num_edges;
        }

        if (num_edges == 0) {
            optimizer.removeVertex(lm_vtx);
            is_optimized_lm.at(i) = false;
        }
    }

    // Connect marker vertices
    for (unsigned int marker_idx = 0; marker_idx < markers.size(); ++marker_idx) {
        auto mkr = markers.at(marker_idx);
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
                reproj_edge_wraps.push_back(reproj_edge_wrap);
                optimizer.addEdge(reproj_edge_wrap.edge_);
            }
        }
    }

    // 5. Perform optimization

    optimizer.initializeOptimization();
    optimizer.optimize(num_iter);

    if (force_stop_flag && *force_stop_flag) {
        return;
    }
}

global_bundle_adjuster::global_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter, const bool use_huber_kernel)
    : map_db_(map_db), num_iter_(num_iter), use_huber_kernel_(use_huber_kernel) {}

void global_bundle_adjuster::optimize_for_initialization(bool* const force_stop_flag) const {
    // 1. Collect the dataset
    auto keyfrms = map_db_->get_all_keyframes();
    auto lms = map_db_->get_all_landmarks();
    auto markers = map_db_->get_all_markers();
    std::vector<bool> is_optimized_lm(lms.size(), true);

    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    // Container of the shot vertices
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, keyfrms.size());
    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, lms.size());
    // Container of the landmark vertices
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, markers.size());
    // Camera intrinsics vertex
    internal::bfx_camera_intrinsics_vertex* camera_intrinsics_vtx = create_camera_intrinsics_vertex(vtx_id_offset, keyfrms);

    g2o::SparseOptimizer optimizer;

    optimize_impl(optimizer, keyfrms, lms, markers, is_optimized_lm, keyfrm_vtx_container, lm_vtx_container,
                  marker_vtx_container, camera_intrinsics_vtx,
                  num_iter_, use_huber_kernel_, force_stop_flag);

    if (force_stop_flag && *force_stop_flag) {
        return;
    }

    // 6. Extract the result

    bool focal_length_modified = populate_camera_from_vertex(keyfrms, camera_intrinsics_vtx);

    //std::cout << settings << std::endl;

    for (auto keyfrm : keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());

        keyfrm->set_pose_cw(cam_pose_cw);
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        const auto& lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        lm->set_pos_in_world(pos_w);
        lm->update_mean_normal_and_obs_scale_variance();
    }
}

bool global_bundle_adjuster::optimize(std::unordered_set<unsigned int>& optimized_keyfrm_ids,
                                      std::unordered_set<unsigned int>& optimized_landmark_ids,
                                      eigen_alloc_unord_map<unsigned int, Vec3_t>& lm_to_pos_w_after_global_BA,
                                      eigen_alloc_unord_map<unsigned int, Mat44_t>& keyfrm_to_pose_cw_after_global_BA,
                                      bool* const force_stop_flag) const {
    // 1. Collect the dataset
    auto keyfrms = map_db_->get_all_keyframes();
    auto lms = map_db_->get_all_landmarks();
    auto markers = map_db_->get_all_markers();
    std::vector<bool> is_optimized_lm(lms.size(), true);

    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    // Container of the shot vertices
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset, keyfrms.size());
    // Container of the landmark vertices
    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, lms.size());
    // Container of the landmark vertices
    internal::marker_vertex_container marker_vtx_container(vtx_id_offset, markers.size());
    // Camera intrinsics vertex
    internal::bfx_camera_intrinsics_vertex* camera_intrinsics_vtx = create_camera_intrinsics_vertex(vtx_id_offset, keyfrms);

    g2o::SparseOptimizer optimizer;
    auto terminateAction = new terminate_action;
    terminateAction->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);

    optimize_impl(optimizer, keyfrms, lms, markers, is_optimized_lm, keyfrm_vtx_container, lm_vtx_container,
                  marker_vtx_container, camera_intrinsics_vtx,
                  num_iter_, use_huber_kernel_, force_stop_flag);

    if (force_stop_flag && *force_stop_flag && !terminateAction->stopped_by_terminate_action_) {
        return false;
    }

    // 6. Extract the result

    bool focal_length_modified = populate_camera_from_vertex(keyfrms, camera_intrinsics_vtx);

    for (auto keyfrm : keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());

        keyfrm_to_pose_cw_after_global_BA[keyfrm->id_] = cam_pose_cw;
        optimized_keyfrm_ids.insert(keyfrm->id_);
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        const auto& lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        lm_to_pos_w_after_global_BA[lm->id_] = pos_w;
        optimized_landmark_ids.insert(lm->id_);
    }

    return true;
}

} // namespace optimize
} // namespace stella_vslam
