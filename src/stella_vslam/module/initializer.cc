#include "stella_vslam/config.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"
#include "stella_vslam/data/marker.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/data/keyframe_autocalibration_wrapper.h"
#include "stella_vslam/initialize/bearing_vector.h"
#include "stella_vslam/initialize/perspective.h"
#include "stella_vslam/marker_model/base.h"
#include "stella_vslam/match/area.h"
#include "stella_vslam/match/prematched.h"
#include "stella_vslam/module/initializer.h"
#include "stella_vslam/optimize/global_bundle_adjuster.h"
#include "stella_vslam/util/video_evaluation.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

namespace stella_vslam {
namespace module {

initializer::initializer(data::map_database* map_db, data::bow_database* bow_db,
                         const stella_vslam_bfx::config_settings& settings)
    : map_db_(map_db), bow_db_(bow_db),
      num_ransac_iters_(settings.num_ransac_iterations_),
      min_num_valid_pts_(settings.min_num_valid_pts_),
      min_num_triangulated_pts_(settings.min_num_triangulated_pts_),
      parallax_deg_thr_(settings.parallax_deg_threshold_),
      reproj_err_thr_(settings.reprojection_error_threshold_),
      num_ba_iters_(settings.num_ba_iterations_),
      scaling_factor_(settings.scaling_factor_),
      use_fixed_seed_(settings.use_fixed_seed_),
      use_orb_features_(settings.use_orb_features_) {
    spdlog::debug("CONSTRUCT: module::initializer");
}

initializer::~initializer() {
    spdlog::debug("DESTRUCT: module::initializer");
}

void initializer::reset() {
    initializer_.reset(nullptr);
    state_ = initializer_state_t::NotReady;
    init_frm_id_ = 0;
    init_frm_stamp_ = 0.0;
    init_matches_.clear();
}

initializer_state_t initializer::get_state() const {
    return state_;
}

unsigned int initializer::get_initial_frame_id() const {
    return init_frm_id_;
}

double initializer::get_initial_frame_timestamp() const {
    return init_frm_stamp_;
}

bool initializer::get_use_fixed_seed() const {
    return use_fixed_seed_;
}

bool check_keyframe_landmarks(data::keyframe const& cur_keyfrm)
{
    const auto cur_lms = cur_keyfrm.get_landmarks();
    for (unsigned int idx = 0; idx < cur_lms.size(); ++idx) {
        auto lm = cur_lms.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        using namespace data;
        landmark::observations_t observations = lm->get_observations();
        Vec3_t mean_normal = Vec3_t::Zero();
        for (const auto& observation : observations) {
            auto keyfrm = observation.first.lock();
            if (!keyfrm)
                return false;
        }
    }
    return true;
}

bool check_keyframe_landmarks(std::shared_ptr<data::keyframe> const& keyfrm) {
   if (!keyfrm)
      return true;
   const auto cur_lms = keyfrm->get_landmarks();
   for (unsigned int idx = 0; idx < cur_lms.size(); ++idx) {
      auto lm = cur_lms.at(idx);
      if (!lm) {
            return true;
      }
      if (lm->will_be_erased()) {
            return true;
      }
      using namespace data;
      landmark::observations_t observations = lm->get_observations();
      Vec3_t mean_normal = Vec3_t::Zero();
      for (const auto& observation : observations) {
            auto keyfrm2 = observation.first.lock();
            if (!keyfrm2)
               return false;
      }
   }
   return true;
}


bool check_keyframe_landmarks(data::map_database* map_db)
{
    auto keyfrms = map_db->get_all_keyframes();
    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm)
            continue;
        const auto cur_lms = keyfrm->get_landmarks();
        for (unsigned int idx = 0; idx < cur_lms.size(); ++idx) {
            auto lm = cur_lms.at(idx);
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }
            using namespace data;
            landmark::observations_t observations = lm->get_observations();
            Vec3_t mean_normal = Vec3_t::Zero();
            for (const auto& observation : observations) {
                auto keyfrm2 = observation.first.lock();
                if (!keyfrm2)
                    return false;
            }
        }
    }
    return true;
}

bool initializer::initialize(const camera::setup_type_t setup_type,
                             data::bow_vocabulary* bow_vocab, data::frame& curr_frm) {
    switch (setup_type) {
        case camera::setup_type_t::Monocular: {
            // construct an initializer if not constructed
            if (state_ == initializer_state_t::NotReady) {
                create_initializer(curr_frm);
                return false;
            }

            bool optimise_focal_length = curr_frm.camera_->autocalibration_parameters_.optimise_focal_length;
            double last_focal_length = stella_vslam_bfx::getCameraFocalLengthXPixels(curr_frm.camera_);
//            bool refine_initialisation(false);
            bool refine_initialisation(optimise_focal_length);
            bool destroy_initialiser_in_createMap(!refine_initialisation);
            data::frame start_init_frm, start_curr_frm;
            if (refine_initialisation) {
                start_init_frm = init_frm_;
                start_curr_frm = curr_frm;
            }

            //initialize::initialisation_cache init_cache;
            //initialize::initialisation_cache* cache(refine_initialisation ? &init_cache : nullptr);

            // try to initialize
            if (!try_initialize_for_monocular(curr_frm, 1.0)) {
               // failed
               return false;
            }

            // create new map if succeeded
            create_map_for_monocular(bow_vocab, curr_frm, destroy_initialiser_in_createMap, optimise_focal_length);

            bool map_check_1 = check_keyframe_landmarks(map_db_);
            if (!map_check_1)
                int y = 0;

            // Try to improve the initialisation now that the focal length estimate has been improved
            if (refine_initialisation) {
               double const focal_length_change_percent_threshold(5.0); /// Stop iterating if the change percent falls below this
               for (int i = 0; i <4; ++i) {
                   double new_focal_length = stella_vslam_bfx::getCameraFocalLengthXPixels(curr_frm.camera_);
                   double focal_length_change_percent = fabs(100.0 * (new_focal_length - last_focal_length) / last_focal_length);
                   last_focal_length = new_focal_length;
                   if (focal_length_change_percent < focal_length_change_percent_threshold)
                       break;
#if 1
                   //init_frm_ = start_init_frm;
                   //curr_frm = start_curr_frm;
                   init_frm_.ref_keyfrm_.reset();
                   curr_frm.ref_keyfrm_.reset();
                   init_frm_.invalidate_pose();
                   curr_frm.invalidate_pose();
                   for (unsigned int idx = 0; idx < curr_frm.landmarks_.size(); ++idx) {
                       curr_frm.landmarks_[idx].reset();
                   }

                   data::frame init_frm = init_frm_; // store the init frame
                   reset(); // reset the initialiser (this)
                   map_db_->clear(); // reset the map_db
                   create_initializer(init_frm); // create a new initialiser
                   bool ok_initialize = try_initialize_for_monocular(curr_frm, 0.7); // Reinitialise with the new focal length
                                                                                       // and lower parallax threshold
                   if (ok_initialize)
                       create_map_for_monocular(bow_vocab, curr_frm, destroy_initialiser_in_createMap, optimise_focal_length);

                 bool map_check_2 = check_keyframe_landmarks(map_db_);
                   if (!map_check_2)
                        int y = 0;
#else

                   if (!refine_initialize_for_monocular(curr_frm, cache)) {
                       // failed
                       break;
                   }

                   //break;

                   // create new map if succeeded
                   map_db_->clear();
                   state_ = initializer_state_t::Initializing;
                   create_map_for_monocular(bow_vocab, curr_frm, destroy_initialiser_in_createMap, optimise_focal_length);
#endif
               }
            }

            //nlohmann::json json_keyfrms, json_landmarks;
            //map_db_->to_json(json_keyfrms, json_landmarks);
            //std::ofstream file_keyfrms("keyframes.json"), file_landmarks("landmarks.json");
            //file_keyfrms << std::setw(4) << json_keyfrms << std::endl;
            //file_keyfrms.close();
            //file_landmarks << std::setw(4) << json_landmarks << std::endl;
            //file_landmarks.close();

            if (!destroy_initialiser_in_createMap)
                initializer_.reset(nullptr);

            break;
        }
        case camera::setup_type_t::Stereo:
        case camera::setup_type_t::RGBD: {
            state_ = initializer_state_t::Initializing;

            // try to initialize
            if (!try_initialize_for_stereo(curr_frm)) {
                // failed
                return false;
            }

            // create new map if succeeded
            create_map_for_stereo(bow_vocab, curr_frm);
            break;
        }
        default: {
            throw std::runtime_error("Undefined camera setup");
        }
    }

    // check the state is succeeded or not
    if (state_ == initializer_state_t::Succeeded) {
        init_frm_id_ = curr_frm.id_;
        init_frm_stamp_ = curr_frm.timestamp_;
        return true;
    }
    else {
        return false;
    }
}

void initializer::create_initializer(data::frame& curr_frm) {
    // set the initial frame
    init_frm_ = data::frame(curr_frm);

    // initialize the previously matched coordinates
    prev_matched_coords_.resize(init_frm_.frm_obs_.undist_keypts_.size());
    for (unsigned int i = 0; i < init_frm_.frm_obs_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = init_frm_.frm_obs_.undist_keypts_.at(i).pt;
    }

    // initialize matchings (init_idx -> curr_idx)
    std::fill(init_matches_.begin(), init_matches_.end(), -1);

    // build a initializer
    initializer_.reset(nullptr);
    switch (init_frm_.camera_->model_type_) {
        case camera::model_type_t::Perspective:
        case camera::model_type_t::Fisheye:
        case camera::model_type_t::RadialDivision: {
            initializer_ = std::unique_ptr<initialize::perspective>(
                new initialize::perspective(
                    init_frm_, num_ransac_iters_, min_num_triangulated_pts_, min_num_valid_pts_,
                    parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));
            break;
        }
        case camera::model_type_t::Equirectangular: {
            initializer_ = std::unique_ptr<initialize::bearing_vector>(
                new initialize::bearing_vector(
                    init_frm_, num_ransac_iters_, min_num_triangulated_pts_, min_num_valid_pts_,
                    parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));
            break;
        }
    }

    state_ = initializer_state_t::Initializing;
}

bool initializer::try_initialize_for_monocular(data::frame& curr_frm, double parallax_deg_thr_multiplier) {
    assert(state_ == initializer_state_t::Initializing);

    unsigned int num_matches = 0;
    if (use_orb_features_) {
        match::area matcher(0.9, true);
        num_matches = matcher.match_in_consistent_area(init_frm_, curr_frm, prev_matched_coords_, init_matches_, 100);
    }
    num_matches += stella_vslam_bfx::get_frames_prematches(init_frm_, curr_frm, prev_matched_coords_, init_matches_);

    if (num_matches < min_num_valid_pts_) {
        // rebuild the initializer with the next frame
        reset();
        return false;
    }

    // try to initialize with the initial frame and the current frame
    assert(initializer_);
    spdlog::debug("try to initialize with the initial frame and the current frame: frame {} - frame {}", init_frm_.id_, curr_frm.id_);
    return initializer_->initialize(curr_frm, init_matches_, parallax_deg_thr_multiplier);
}

#if 0
bool initializer::refine_initialize_for_monocular(data::frame& curr_frm, initialize::initialisation_cache* cache) {


   #if 1 // complete reset

initializer_.reset(nullptr);

{ // create_initialiser without setting the init frame to current

       // initialize the previously matched coordinates
    prev_matched_coords_.resize(init_frm_.frm_obs_.undist_keypts_.size());
    for (unsigned int i = 0; i < init_frm_.frm_obs_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = init_frm_.frm_obs_.undist_keypts_.at(i).pt;
    }

    // initialize matchings (init_idx -> curr_idx)
    std::fill(init_matches_.begin(), init_matches_.end(), -1);

    // build a initializer
    initializer_.reset(nullptr);
    switch (init_frm_.camera_->model_type_) {
        case camera::model_type_t::Perspective:
        case camera::model_type_t::Fisheye:
        case camera::model_type_t::RadialDivision: {
            initializer_ = std::unique_ptr<initialize::perspective>(
                new initialize::perspective(
                    init_frm_, num_ransac_iters_, min_num_triangulated_, min_num_valid_pts_,
                    parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));
            break;
        }
        case camera::model_type_t::Equirectangular: {
            initializer_ = std::unique_ptr<initialize::bearing_vector>(
                new initialize::bearing_vector(
                    init_frm_, num_ransac_iters_, min_num_triangulated_, min_num_valid_pts_,
                    parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));
            break;
        }
    }

    state_ = initializer_state_t::Initializing;
}

bool init = try_initialize_for_monocular(curr_frm, cache);
return init;
   #else

   if (init_matches_.empty())
        return false;

    // try to initialize with the initial frame and the current frame
    assert(initializer_);
    spdlog::debug("refine initialization with the initial frame and the current frame: frame {} - frame {}", init_frm_.id_, curr_frm.id_);
    bool init = initializer_->cached_initialize(curr_frm, init_matches_, cache);
    return init;
    #endif
}
#endif

bool initializer::create_map_for_monocular(data::bow_vocabulary* bow_vocab, data::frame& curr_frm, bool destroy_initialiser, bool optimise_focal_length) {
    assert(state_ == initializer_state_t::Initializing);

    eigen_alloc_vector<Vec3_t> init_triangulated_pts;
    {
        assert(initializer_);
        init_triangulated_pts = initializer_->get_triangulated_pts();
        const auto is_triangulated = initializer_->get_triangulated_flags();

        // make invalid the matchings which have not been triangulated
        for (unsigned int i = 0; i < init_matches_.size(); ++i) {
            if (init_matches_.at(i) < 0) {
                continue;
            }
            if (is_triangulated.at(i)) {
                continue;
            }
            init_matches_.at(i) = -1;
        }

        // set the camera poses
        init_frm_.set_pose_cw(Mat44_t::Identity());
        Mat44_t cam_pose_cw = Mat44_t::Identity();
        cam_pose_cw.block<3, 3>(0, 0) = initializer_->get_rotation_ref_to_cur();
        cam_pose_cw.block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();
        curr_frm.set_pose_cw(cam_pose_cw);

        // destruct the initializer
        if (destroy_initialiser)
            initializer_.reset(nullptr);
    }

    // create initial keyframes
    auto init_keyfrm = data::keyframe::make_keyframe(map_db_->next_keyframe_id_++, init_frm_);
    auto curr_keyfrm = data::keyframe::make_keyframe(map_db_->next_keyframe_id_++, curr_frm);
    curr_keyfrm->graph_node_->set_spanning_parent(init_keyfrm);
    init_keyfrm->graph_node_->add_spanning_child(curr_keyfrm);
    init_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
    curr_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
    map_db_->add_spanning_root(init_keyfrm);

    // compute BoW representations
    init_keyfrm->compute_bow(bow_vocab);
    curr_keyfrm->compute_bow(bow_vocab);


bool map_check_d = check_keyframe_landmarks(map_db_);
    if (!map_check_d)
        int y = 0;

    // add the keyframes to the map DB
    map_db_->add_keyframe(init_keyfrm);
    bool map_check_e = check_keyframe_landmarks(map_db_);
    if (!map_check_e)
        int y = 0;

    map_db_->add_keyframe(curr_keyfrm);
    bool map_check_f = check_keyframe_landmarks(map_db_);
    if (!map_check_f)
        int y = 0;

    // update the frame statistics
    init_frm_.ref_keyfrm_ = init_keyfrm;
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(init_frm_, false);
    map_db_->update_frame_statistics(curr_frm, false);

bool map_check_c = check_keyframe_landmarks(map_db_);
if (!map_check_c)
        int y = 0;

    // assign 2D-3D associations
    std::vector<std::shared_ptr<data::landmark>> lms;
    for (unsigned int init_idx = 0; init_idx < init_matches_.size(); init_idx++) {
        const auto curr_idx = init_matches_.at(init_idx);
        if (curr_idx < 0) {
            continue;
        }

        // construct a landmark
        auto lm = std::make_shared<data::landmark>(map_db_->next_landmark_id_++, init_triangulated_pts.at(init_idx), curr_keyfrm);

        // set the assocications to the new keyframes
        lm->connect_to_keyframe(init_keyfrm, init_idx);
        lm->connect_to_keyframe(curr_keyfrm, curr_idx);

        // update the descriptor
        lm->compute_descriptor();
        // update the geometry
        lm->update_mean_normal_and_obs_scale_variance();

        // temp
        data::landmark::observations_t observations = lm->get_observations();
        Vec3_t mean_normal = Vec3_t::Zero();
        for (const auto& observation : observations) {
            auto keyfrm = observation.first.lock();
            if (!keyfrm)
                int y = 0;
        }
        // temp

        // set the 2D-3D assocications to the current frame
        curr_frm.add_landmark(lm, curr_idx);

        // add the landmark to the map DB
        map_db_->add_landmark(lm);
        lms.push_back(lm);
    }

bool map_check_a = check_keyframe_landmarks(map_db_);
if (!map_check_a)
   int y = 0;

    bool indefinite_scale = true;
    for (const auto& id_mkr2d : init_keyfrm->markers_2d_) {
        if (curr_keyfrm->markers_2d_.count(id_mkr2d.first)) {
            indefinite_scale = false;
            break;
        }
    }

    // assign marker associations
    std::vector<std::shared_ptr<data::marker>> markers;
    const auto assign_marker_associations = [this, &markers](const std::shared_ptr<data::keyframe>& keyfrm) {
        for (const auto& id_mkr2d : keyfrm->markers_2d_) {
            auto marker = map_db_->get_marker(id_mkr2d.first);
            if (!marker) {
                auto mkr2d = id_mkr2d.second;
                eigen_alloc_vector<Vec3_t> corners_pos_w = mkr2d.compute_corners_pos_w(keyfrm->get_pose_wc(), mkr2d.marker_model_->corners_pos_);
                marker = std::make_shared<data::marker>(corners_pos_w, id_mkr2d.first, mkr2d.marker_model_);
                // add the marker to the map DB
                map_db_->add_marker(marker);
                markers.push_back(marker);
            }
            // Set the association to the new marker
            keyfrm->add_marker(marker);
            marker->observations_.push_back(keyfrm);
        }
    };
    assign_marker_associations(init_keyfrm);
    assign_marker_associations(curr_keyfrm);

    // global bundle adjustment
    const auto global_bundle_adjuster = optimize::global_bundle_adjuster(num_ba_iters_, true);
	std::vector<std::shared_ptr<data::keyframe>> keyfrms{init_keyfrm, curr_keyfrm};
    bool *const null_force_stop_flag(nullptr), camera_was_modified;
    global_bundle_adjuster.optimize_for_initialization(keyfrms, lms, markers, null_force_stop_flag, &camera_was_modified);
    if (camera_was_modified) {
        curr_frm.frm_obs_.bearings_.clear();
        curr_frm.camera_->convert_keypoints_to_bearings(curr_frm.frm_obs_.undist_keypts_, curr_frm.frm_obs_.bearings_);
        init_frm_.frm_obs_.bearings_.clear();
        init_frm_.camera_->convert_keypoints_to_bearings(init_frm_.frm_obs_.undist_keypts_, init_frm_.frm_obs_.bearings_);
    }

    if (indefinite_scale) {
        // scale the map so that the median of depths is 1.0
        const auto median_depth = init_keyfrm->compute_median_depth(init_keyfrm->camera_->model_type_ == camera::model_type_t::Equirectangular);
        const auto inv_median_depth = 1.0 / median_depth;
        if (curr_keyfrm->get_num_tracked_landmarks(1) < min_num_triangulated_pts_ && median_depth < 0) {
            spdlog::info("seems to be wrong initialization, resetting");
            state_ = initializer_state_t::Wrong;
            return false;
        }
        scale_map(init_keyfrm, curr_keyfrm, inv_median_depth * scaling_factor_);
    }

    // update the current frame pose
    curr_frm.set_pose_cw(curr_keyfrm->get_pose_cw());

    spdlog::info("new map created with {} points: frame {} - frame {}", map_db_->get_num_landmarks(), init_frm_.id_, curr_frm.id_);
    state_ = initializer_state_t::Succeeded;
    return true;
}

void initializer::scale_map(const std::shared_ptr<data::keyframe>& init_keyfrm, const std::shared_ptr<data::keyframe>& curr_keyfrm, const double scale) {
    // scaling keyframes
    Mat44_t cam_pose_cw = curr_keyfrm->get_pose_cw();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    curr_keyfrm->set_pose_cw(cam_pose_cw);

    // scaling landmarks
    const auto landmarks = init_keyfrm->get_landmarks();
    for (const auto& lm : landmarks) {
        if (!lm) {
            continue;
        }
        lm->set_pos_in_world(lm->get_pos_in_world() * scale);
        lm->update_mean_normal_and_obs_scale_variance();
    }
}

bool initializer::try_initialize_for_stereo(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);
    // count the number of valid depths
    unsigned int num_valid_depths = std::count_if(curr_frm.frm_obs_.depths_.begin(), curr_frm.frm_obs_.depths_.end(),
                                                  [](const float depth) {
                                                      return 0 < depth;
                                                  });
    return min_num_triangulated_pts_ <= num_valid_depths;
}

bool initializer::create_map_for_stereo(data::bow_vocabulary* bow_vocab, data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);

    // create an initial keyframe
    curr_frm.set_pose_cw(Mat44_t::Identity());
    auto curr_keyfrm = data::keyframe::make_keyframe(map_db_->next_keyframe_id_++, curr_frm);
    curr_keyfrm->graph_node_->set_spanning_root(curr_keyfrm);
    map_db_->add_spanning_root(curr_keyfrm);

    // compute BoW representation
    curr_keyfrm->compute_bow(bow_vocab);

    // add to the map DB
    map_db_->add_keyframe(curr_keyfrm);

    // update the frame statistics
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(curr_frm, false);

    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.num_keypts_; ++idx) {
        // add a new landmark if tht corresponding depth is valid
        const auto z = curr_frm.frm_obs_.depths_.at(idx);
        if (z <= 0) {
            continue;
        }

        // build a landmark
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = std::make_shared<data::landmark>(map_db_->next_landmark_id_++, pos_w, curr_keyfrm);

        // set the associations to the new keyframe
        lm->connect_to_keyframe(curr_keyfrm, idx);

        // update the descriptor
        lm->compute_descriptor();
        // update the geometry
        lm->update_mean_normal_and_obs_scale_variance();

        // set the 2D-3D associations to the current frame
        curr_frm.add_landmark(lm, idx);

        // add the landmark to the map DB
        map_db_->add_landmark(lm);
    }

    spdlog::info("new map created with {} points: frame {}", map_db_->get_num_landmarks(), curr_frm.id_);
    state_ = initializer_state_t::Succeeded;
    return true;
}

} // namespace module
} // namespace stella_vslam
