#include "solver.h"

#include <opencv2/core/mat.hpp>

#include <spdlog/spdlog.h>

#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/map_database.h>
#include <stella_vslam/data/landmark.h>

#include "type.h"
#include "system.h"

using namespace stella_vslam;

namespace stella_vslam_bfx {

solver::solver(const std::shared_ptr<config>& cfg,
               const std::string& vocab_file_path,
               std::function<bool(int, cv::Mat&)> get_frame)
: get_frame_(get_frame)
{
    spdlog::debug("debug log message test solver");
    spdlog::info("info log message test solver");
//    spdlog::set_level(spdlog::level::from_str(log_level->value()));
    // build the slam system
    slam_ = std::make_shared<stella_vslam::system>(cfg, vocab_file_path);

    spdlog::debug("debug log message test solver 2");
    spdlog::info("info log message test solver 2");


    bool already_have_map(false); // see run_video_slam for the case where we already have a map
    slam_->startup(!already_have_map);
}

solver::~solver()
{
   // shutdown the slam process
   if (slam_)
      slam_->shutdown();
}

void solver::set_progress_callback(std::function<void(float)> set_progress)
{
   set_progress_ = set_progress;
}

void solver::set_stage_description_callback(std::function<void(std::string)> set_stage_description)
{
   set_stage_description_ = set_stage_description;
}

void solver::set_display_frame_callback(std::function<void(std::shared_ptr<frame_display_data>)> display_frame) {
   display_frame_ = display_frame;
}

void solver::set_cancel_callback(std::function<bool()> cancel)
{
   cancel_ = cancel;
}

static std::tuple<std::shared_ptr<data::keyframe>, int> earliest_valid_keyframe(std::vector<std::shared_ptr<data::keyframe>> const& keyfrms,
                                                                                       std::map<double, int> const& timestampToVideoFrame) {
    if (!keyfrms.empty()) {
        std::shared_ptr<data::keyframe> first_keyframe_data = *std::min_element(keyfrms.begin(), keyfrms.end(),
                                                                                              [](const auto& a, const auto& b) { // earliest above landmark threshold
                                                                                                  int landmarkThreshold(10);
                                                                                                  bool validLandmarksA(a->get_valid_landmarks().size() >= landmarkThreshold);
                                                                                                  bool validLandmarksB(b->get_valid_landmarks().size() >= landmarkThreshold);
                                                                                                  if (!validLandmarksA && validLandmarksB)
                                                                                                      return false;
                                                                                                  if (validLandmarksA && !validLandmarksB)
                                                                                                      return true;
                                                                                                  return a->timestamp_ < b->timestamp_;
                                                                                              });
        auto f = timestampToVideoFrame.find(first_keyframe_data->timestamp_);
        if (f != timestampToVideoFrame.end())
            return {first_keyframe_data, f->second};
    }
    return {nullptr, -1};
}

double overall_progress_percent(double stage_progress, double stage_start_progress, double stage_end_progress)
{
    return 100.0 * (stage_start_progress + stage_progress * (stage_end_progress - stage_start_progress));
}

bool solver::track_frame_range(int begin, int end, tracking_direction direction, solve* final_solve) {

   cv::Mat frame_image;
   cv::Mat mask; // to do!!
   prematched_points* extra_keypoints(nullptr); // to do!!

   if (!get_frame_)
       return false; // report error!

   std::map<double, int> timestampToVideoFrame;

   double timestamp(0);

   // To do, support the tracking direction
   if (direction == tracking_direction_backwards)
       return false; 

   if (set_stage_description_)
       set_stage_description_("Analysing video");
   if (set_progress_)
       set_progress_(0);

   // Track forwards to create the map
   for (int frame = begin; frame <= end; ++frame) {

      bool got_frame = get_frame_(frame, frame_image);
      if (!got_frame || frame_image.empty())
         continue;
      
      timestamp = (double)frame;

      timestampToVideoFrame[timestamp] = frame;

      slam_->feed_monocular_frame(frame_image, timestamp, mask, extra_keypoints);

      double stage_progress = double(frame - begin + 1) / double(1 + end - begin);
      if (set_progress_)
          set_progress_(overall_progress_percent(stage_progress, 0.0, 0.4));
      if (cancel_ && cancel_())
          return false;

   }

   // Find the initialisation point (the first keyframe in the map)
   // Relocalise the tracker to the initialisation point's pose
   auto [first_keyframe_data, first_keyframe] = earliest_valid_keyframe(slam_->map_db_->get_all_keyframes(), timestampToVideoFrame);
   if (first_keyframe_data) {
       bool resultRelocalise = slam_->relocalize_by_pose(first_keyframe_data->get_pose_wc());
       if (resultRelocalise == false)
           spdlog::warn("Failed to relocalise to first keyframe pose (video frame {})", first_keyframe);
   }
   spdlog::info("Relocalised to first keyframe pose (video frame {})", first_keyframe);

   // Some bookkeeping
   std::vector<double> videoFrameToTimestamp(end+1, -1.0);
   for (auto const& tf : timestampToVideoFrame)
       videoFrameToTimestamp[tf.second] = tf.first;

   if (set_stage_description_)
      set_stage_description_("Revisiting video");

   // Track backwards from the initialization point to finish the map
   for (int frame = first_keyframe - 1; frame >= begin; --frame) {
       bool got_frame = get_frame_(frame, frame_image);
       if (!got_frame || frame_image.empty())
           continue;

       timestamp = videoFrameToTimestamp[frame];

       slam_->feed_monocular_frame(frame_image, timestamp, mask, extra_keypoints);

       double stage_progress = double(first_keyframe - frame) / double(first_keyframe - begin);
       if (set_progress_)
           set_progress_(overall_progress_percent(stage_progress, 0.4, 0.5));
       if (cancel_ && cancel_())
           return false;

   }
   // Wait for map optimisation after loop closing to finish
   while (slam_->loop_BA_is_running() || !slam_->mapping_module_is_enabled()) {
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }

   if (set_stage_description_)
       set_stage_description_("Optimising");

   // Optimise the map
   if (true) {
       slam_->run_loop_BA();
       // Wait for map optimisation to finish
       while (slam_->loop_BA_is_running() || !slam_->mapping_module_is_enabled()) {
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
       }
   }

   if (set_stage_description_)
      set_stage_description_("Tracking");

   // Track forwards (without mapping) to calculate the non-keyframe camera positions
   slam_->disable_mapping_module();
   for (int frame = begin; frame <= end; ++frame) {
       bool got_frame = get_frame_(frame, frame_image);
       if (!got_frame || frame_image.empty())
           continue;

       timestamp = videoFrameToTimestamp[frame];

       auto camera_pose = slam_->feed_monocular_frame(frame_image, timestamp, mask, extra_keypoints);

       // Store the camera pose
       if (camera_pose && final_solve)
           final_solve->frame_to_camera[frame] = *camera_pose;

       double stage_progress = double(frame - begin + 1) / double(1 + end - begin);
       if (set_progress_)
           set_progress_(overall_progress_percent(stage_progress, 0.7, 0.99));
       if (cancel_ && cancel_())
           return false;
   }

   // Store the camera intrinsics (lens properties)
   if (final_solve) {
      // Get the camera from the keyframes (maybe there's a better place?)
      auto keyfrms = slam_->map_db_->get_all_keyframes();
      for (const auto& keyfrm : keyfrms) {
         if (keyfrm && !keyfrm->will_be_erased() && keyfrm->camera_->model_type_ == camera::model_type_t::Perspective) {
            final_solve->camera_lens = std::make_shared<camera::perspective> (*static_cast<camera::perspective*>(keyfrm->camera_));
            break;
         }
      }
   }

   // Store the map points
   if (final_solve) {
       data::map_database const* map_db = slam_->map_db_;
       std::vector<std::shared_ptr<data::landmark>> all_landmarks = map_db->get_all_landmarks();
       final_solve->world_points.clear();
       std::transform(all_landmarks.cbegin(), all_landmarks.cend(), std::back_inserter(final_solve->world_points),
                      [](const std::shared_ptr<data::landmark>& value) { return value->get_pos_in_world(); });
   }

   return true;
}

std::shared_ptr<stella_vslam::system> solver::system()
{
    return slam_;
}

}

