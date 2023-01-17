#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "stella_vslam/system.h"
#include "stella_vslam/config.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/util/yaml.h"
#include "stella_vslam/util/video_evaluation.h"
#include "util/tinyxml2.h"


#include "stella_vslam/data/map_database.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"

#include <iostream>
#include <chrono>
#include <fstream>
#include <numeric>
#include <dirent.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <opencv2/core/types.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <backward.hpp>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

// TODO move to shared location for other examples to use
stella_vslam_bfx::config_settings * settings_from_yaml(YAML::Node yaml_node)
{
    stella_vslam_bfx::config_settings * settings = nullptr;

    // Essential camera settings
    const auto camera_node = yaml_node["Camera"];

    stella_vslam::camera::model_type_t camera_model;
    const auto model_str = camera_node["model"].as<std::string>();
    if (model_str == "perspective")
        camera_model = stella_vslam::camera::model_type_t::Perspective;
    else if (model_str == "fisheye")
        camera_model = stella_vslam::camera::model_type_t::Fisheye;
    else if (model_str == "equirectangular")
        camera_model = stella_vslam::camera::model_type_t::Equirectangular;
    else if (model_str == "radial_division")
        camera_model = stella_vslam::camera::model_type_t::RadialDivision;
    else
        throw std::runtime_error("Invalid camera model: " + model_str);

    stella_vslam::camera::setup_type_t camera_setup;
    const auto setup_type_str = camera_node["setup"].as<std::string>();
    if (setup_type_str == "monocular")
        camera_setup = stella_vslam::camera::setup_type_t::Monocular;
    else if (setup_type_str == "stereo")
        camera_setup = stella_vslam::camera::setup_type_t::Stereo;
    else if (setup_type_str == "RGBD")
        camera_setup = stella_vslam::camera::setup_type_t::RGBD;
    else
        throw std::runtime_error("Invalid setup type: " + setup_type_str);
    
    stella_vslam::camera::color_order_t colour_order = stella_vslam::camera::color_order_t::Gray;
    if (camera_node["color_order"])
    {
        const auto colour_order_str = camera_node["color_order"].as<std::string>();
        if (colour_order_str == "Gray")
            colour_order = stella_vslam::camera::color_order_t::Gray;
        else if (colour_order_str == "RGB" || colour_order_str == "RGBA")
            colour_order = stella_vslam::camera::color_order_t::RGB;
        else if (colour_order_str == "BGR" || colour_order_str == "BGRA")
            colour_order = stella_vslam::camera::color_order_t::BGR;
        else
            throw std::runtime_error("Invalid color order: " + colour_order_str);
    }
    
    int cols = camera_node["cols"].as<unsigned int>();
    int rows = camera_node["rows"].as<unsigned int>();
    double fps = camera_node["fps"].as<double>();
    
    switch (camera_model)
    {
        case stella_vslam::camera::model_type_t::Perspective:
        {
            double fx = camera_node["fx"].as<double>();
            double fy = camera_node["fy"].as<double>();
            double cx = camera_node["cx"].as<double>();
            double cy = camera_node["cy"].as<double>();
            double p1 = camera_node["p1"].as<double>();
            double p2 = camera_node["p2"].as<double>();
            double k1 = camera_node["k1"].as<double>();
            double k2 = camera_node["k2"].as<double>();
            double k3 = camera_node["k3"].as<double>();
            
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                                             colour_order, cols, rows, fps,
                                            fx, fy, cx, cy, p1, p2, k1, k2, k3);
            break;
        }
        case stella_vslam::camera::model_type_t::Fisheye:
        {
            double fx = camera_node["fx"].as<double>();
            double fy = camera_node["fy"].as<double>();
            double cx = camera_node["cx"].as<double>();
            double cy = camera_node["cy"].as<double>();
            double k1 = camera_node["k1"].as<double>();
            double k2 = camera_node["k2"].as<double>();
            double k3 = camera_node["k3"].as<double>();
            double k4 = camera_node["k4"].as<double>();
            
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                                             colour_order, cols, rows, fps,
                                            fx, fy, cx, cy, k1, k2, k3, k4);
            break;
        }
        case stella_vslam::camera::model_type_t::RadialDivision:
        {
            double fx = camera_node["fx"].as<double>();
            double fy = camera_node["fy"].as<double>();
            double cx = camera_node["cx"].as<double>();
            double cy = camera_node["cy"].as<double>();
            double d = camera_node["distortion"].as<double>();
            
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                                             colour_order, cols, rows, fps,
                                            fx, fy, cx, cy, d);
            break;
        }
        case stella_vslam::camera::model_type_t::Equirectangular:
        {
            settings = new stella_vslam_bfx::config_settings(camera_model, camera_setup,
                                            colour_order, cols, rows, fps);
            break;
        }
        default:
            break;
    }
    
    if (!settings)
        throw std::runtime_error("Invalid settings");
    
    // Stereo settings
    settings->focal_x_baseline_ = camera_node["focal_x_baseline"].as<double>(0.0);
    settings->depth_threshold_ = camera_node["depth_threshold"].as<double>(40.0);

    // Optional settings //
    using namespace stella_vslam::util;

    // Preprocessing
    const auto preproc_node = yaml_optional_ref(yaml_node, "Preprocessing");
    if ( preproc_node.size() ) {
        settings->max_num_keypoints_ = preproc_node["max_num_keypoints"].as<unsigned int>(2000);
        settings->ini_max_num_keypoints_ = preproc_node["ini_max_num_keypoints"].as<unsigned int>(2 * settings->max_num_keypoints_);
        settings->depthmap_factor_ = preproc_node["depthmap_factor"].as<double>(1.0);
        settings->mask_rectangles_ =
            preproc_node["mask_rectangles"].as<std::vector<std::vector<float>>>(std::vector<std::vector<float>>());
    }

    // ORB
    const auto orb_node = yaml_optional_ref(yaml_node, "Feature");
    if ( orb_node.size() ) {
        settings->use_orb_features_ = orb_node["use_orb_features"].as<bool>(true);
        settings->undistort_prematches_ = orb_node["undistort_prematches"].as<bool>(true);
        settings->ini_fast_threshold_ = orb_node["ini_fast_threshold"].as<unsigned int>(20);
        settings->min_fast_threshold_ = orb_node["min_fast_threshold"].as<unsigned int>(7);
        settings->num_levels_ = orb_node["num_levels"].as<unsigned int>(8);
        settings->scale_factor_ = orb_node["scale_factor"].as<float>(1.2);
    }

    // Marker
    const auto marker_node = yaml_optional_ref(yaml_node, "MarkerModel");
    if ( marker_node.size() ) {
        settings->marker_model_is_enabled_ = true;
        const auto marker_str = marker_node["type"].as<std::string>();
        if (marker_str == "aruco") {
            settings->marker_model_ = stella_vslam::marker_model::model_type_t::Aruco;
            settings->marker_width_ = marker_node["width"].as<double>();
            settings->marker_size_ = marker_node["marker_size"].as<double>();
            settings->max_markers_ = marker_node["max_markers"].as<double>();
        }
        else
            throw std::runtime_error("Invalid marker model type :" + marker_str);
    }

    // Tracking
    const auto track_node = yaml_optional_ref(yaml_node, "Tracking");
    if ( track_node.size() ) {
        settings->enable_auto_relocalization_ = track_node["enable_auto_relocalization"].as<bool>(true);
        settings->use_robust_matcher_for_relocalization_request_ = track_node["use_robust_matcher_for_relocalization_request"].as<bool>(false);
        settings->reloc_distance_threshold_ = track_node["reloc_distance_threshold"].as<double>(0.2);
        settings->reloc_angle_threshold_ = track_node["reloc_angle_threshold"].as<double>(0.45);
    }

    // Mapping
    const auto mapping_node = yaml_optional_ref(yaml_node, "Mapping");
    if ( mapping_node.size() ) {
        if (mapping_node["baseline_dist_thr"]) {
            if (mapping_node["baseline_dist_thr_ratio"]) {
                throw std::runtime_error("Do not set both baseline_dist_thr_ratio and baseline_dist_thr.");
            }
            settings->baseline_dist_thr_ = mapping_node["baseline_dist_thr"].as<double>(1.0);
            settings->use_baseline_dist_thr_ratio_ = false;
        }
        else {
            settings->baseline_dist_thr_ratio_ = mapping_node["baseline_dist_thr_ratio"].as<double>(0.02);
            settings->use_baseline_dist_thr_ratio_ = true;
        }
        settings->use_additional_keyframes_for_monocular_ = mapping_node["use_additional_keyframes_for_monocular"].as<bool>(false);
        settings->num_obs_thr_ = mapping_node["num_obs_thr"].as<unsigned int>(2);
        settings->num_reliable_keyfrms_ = mapping_node["num_reliable_keyfrms"].as<unsigned int>(2);
        settings->desired_valid_obs_ = mapping_node["desired_valid_obs"].as<unsigned int>(0);
        settings->num_obs_keyfrms_thr_ = mapping_node["num_obs_keyframes_thr"].as<unsigned int>(2);
        settings->redundant_obs_ratio_thr_ = mapping_node["redundant_obs_ratio_thr"].as<double>(0.9);
        settings->observed_ratio_thr_ = mapping_node["observed_ratio_thr"].as<double>(0.3);
    }

    // Keyframe
    const auto keyframe_node = yaml_optional_ref(yaml_node, "KeyframeInserter");
    if ( keyframe_node.size() ) {
        settings->enough_lms_thr_ = keyframe_node["enough_lms_thr"].as<unsigned int>(100);
        settings->max_interval_ = keyframe_node["max_interval"].as<double>(1.0);
        settings->min_interval_ = keyframe_node["min_interval"].as<double>(0.1);
        settings->max_distance_ = keyframe_node["max_distance"].as<double>(-1.0);
        settings->lms_ratio_thr_almost_all_lms_are_tracked_ = keyframe_node["lms_ratio_thr_almost_all_lms_are_tracked"].as<double>(0.95);
        settings->lms_ratio_thr_view_changed_ = keyframe_node["lms_ratio_thr_view_changed"].as<double>(0.9);
    }

    // Solve/initializer
    const auto solve_node = yaml_optional_ref(yaml_node, "Initializer");
    if ( solve_node.size() ) {
        settings->use_fixed_seed_= solve_node["use_fixed_seed"].as<bool>(true);
        settings->num_ransac_iterations_= solve_node["num_ransac_iterations"].as<unsigned int>(100);
        settings->min_num_triangulated_pts_ = solve_node["num_min_triangulated_pts"].as<unsigned int>(50);
        settings->num_ba_iterations_= solve_node["num_ba_iterations"].as<unsigned int>(20);
        settings->parallax_deg_threshold_= solve_node["parallax_deg_threshold"].as<float>(1.0);
        settings->reprojection_error_threshold_= solve_node["reprojection_error_threshold"].as<float>(4.0);
        settings->scaling_factor_= solve_node["scaling_factor"].as<float>(1.0);
        settings->optimise_focal_length_ = solve_node["optimise_focal_length"].as<bool>(true);
    }
    
    // Loop detector
    const auto loop_node = yaml_optional_ref(yaml_node, "LoopDetector");
    if ( loop_node.size() ) {
        settings->loop_detector_is_enabled_= loop_node["enabled"].as<bool>(true);
        settings->loop_detector_use_fixed_seed_ = loop_node["use_fixed_seed_"].as<bool>(true);
        settings->reject_by_graph_distance_ = loop_node["reject_by_graph_distance"].as<bool>(false);
        settings->num_final_matches_threshold_= loop_node["num_final_matches_threshold"].as<unsigned int>(40);
        settings->min_continuity_= loop_node["min_continuity"].as<unsigned int>(3);
        settings->min_distance_on_graph_ = loop_node["min_distance_on_graph"].as<unsigned int>(50);
        settings->num_matches_thr_ = loop_node["num_matches_thr"].as<unsigned int>(20);
        settings->num_matches_thr_robust_matcher_ = loop_node["num_matches_thr_robust_matcher"].as<unsigned int>(0);
        settings->num_optimized_inliers_thr_ = loop_node["num_optimized_inliers_thr"].as<unsigned int>(20);
        settings->top_n_covisibilities_to_search_ = loop_node["top_n_covisibilities_to_search"].as<unsigned int>(0);
    }

    // Relocalizer
    const auto reloc_node = yaml_optional_ref(yaml_node, "Relocalizer");
    if ( reloc_node.size() ) {
        settings->relocalizer_use_fixed_seed_ = reloc_node["use_fixed_seed_"].as<bool>(true);
        settings->min_num_bow_matches_ = reloc_node["min_num_bow_matches"].as<unsigned int>(20);
        settings->min_num_valid_obs_=  reloc_node["min_num_valid_obs"].as<unsigned int>(50);
        settings->bow_match_lowe_ratio_ = reloc_node["bow_match_lowe_ratio"].as<double>(0.75);
        settings->proj_match_lowe_ratio_ = reloc_node["proj_match_lowe_ratio"].as<double>(0.9);
        settings->robust_match_lowe_ratio_ = reloc_node["robust_match_lowe_ratio"].as<double>(0.8);
    }

    // Stereo rectifier
    const auto stereo_node = yaml_optional_ref(yaml_node, "StereoRectifier");
    if ( stereo_node.size() ) {
        settings->K_left_ = stereo_node["K_left"].as<std::vector<double>>();
        settings->K_right_ = stereo_node["K_right"].as<std::vector<double>>();
        settings->R_left_ = stereo_node["R_left"].as<std::vector<double>>();
        settings->R_right_ = stereo_node["R_right"].as<std::vector<double>>();
        settings->D_left_ = stereo_node["D_left"].as<std::vector<double>>();
        settings->D_right_ = stereo_node["D_right"].as<std::vector<double>>();
    }

    // System
    const auto system_node = yaml_optional_ref(yaml_node, "System");
    if ( system_node.size() ) {
        const auto map_format_str = system_node["map_format"].as<std::string>();
        if (map_format_str == "sqlite3")
            settings->map_format_ = stella_vslam::io::map_format_t::SQLite3;
        else
            settings->map_format_ = stella_vslam::io::map_format_t::Msgpack;
    }

    std::cout << settings << std::endl;

    return settings;
}

unsigned get_point_id(const std::string& layerName, unsigned i, unsigned j)
{
    // Keep track of point IDs assigned for matching; key is <layer name, grid coordinates>
    static unsigned currPointID = 0;
    static std::map<std::tuple<std::string, unsigned, unsigned>, unsigned> pointIDs;

    std::tuple<std::string, unsigned, unsigned> idKey(layerName, i, j);
    if (pointIDs.find(idKey) == pointIDs.end())
        pointIDs[idKey] = currPointID++;
    return pointIDs.at(idKey);
}

// Reads tracked points from Mocha export file (Quantel xml format) and generates a grid of "feature points" with IDs for matching across frames
void planar_points(const std::string& dirpath, std::map<int, stella_vslam_bfx::prematched_points>& pointsPerFrame, int w, int h, unsigned gridSize = 5)
{
    // Basic point that can be averaged
    struct Coord
    {
        float x, y;
        Coord(float x_ = 0.f, float y_ = 0.f) : x(x_), y(y_) {}

        Coord operator*(const float& f) { return Coord(f * x, f * y); }
        Coord operator+(const Coord& c) { return Coord(x + c.x, y + c.y); }
    };

    // Create a grid for each layer in every frame from the four surface corners to maximise chances of triangulation
    gridSize = std::min(25u, std::max(2u, gridSize));
    const unsigned gridMaxIdx = gridSize - 1;
    const float gridStep = 1.f / float(gridMaxIdx);
    std::vector<std::vector<Coord>> gridPoints(gridSize, std::vector<Coord>(gridSize)); // temporary storage for a layer's point grid
    
    auto dir = opendir(dirpath.c_str());
    struct dirent * file = readdir(dir);
    int frame;
    while (file != nullptr)
    {
        if (file->d_type == DT_REG)
        {
            tinyxml2::XMLDocument doc(true, tinyxml2::COLLAPSE_WHITESPACE);
            doc.LoadFile((dirpath + "/" + file->d_name).c_str());

            auto layerName = doc.FirstChildElement()->FirstChildElement("Object")->FirstChildElement("Name")->FirstChild()->Value();
            auto frameNode = doc.FirstChildElement()->FirstChildElement("Object")->FirstChildElement("Frame");

            while (frameNode)
            {
                frame = std::stoi( frameNode->FirstChildElement("FrameNumber")->FirstChild()->Value() );

                // Corner points from file
                gridPoints[0][0].x = std::min( std::max( std::stof(frameNode->FirstChildElement("topleftx")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[0][0].y = 1.0f - std::min( std::max( std::stof(frameNode->FirstChildElement("toplefty")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[0][gridMaxIdx].x = std::min( std::max( std::stof(frameNode->FirstChildElement("toprightx")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[0][gridMaxIdx].y = 1.0f - std::min( std::max( std::stof(frameNode->FirstChildElement("toprighty")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[gridMaxIdx][0].x = std::min( std::max( std::stof(frameNode->FirstChildElement("bottomleftx")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[gridMaxIdx][0].y = 1.0f - std::min( std::max( std::stof(frameNode->FirstChildElement("bottomlefty")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[gridMaxIdx][gridMaxIdx].x = std::min( std::max( std::stof(frameNode->FirstChildElement("bottomrightx")->FirstChild()->Value()), 0.0f), 1.0f );
                gridPoints[gridMaxIdx][gridMaxIdx].y = 1.0f - std::min( std::max( std::stof(frameNode->FirstChildElement("bottomrighty")->FirstChild()->Value()), 0.0f), 1.0f );

                // Fill in central points
                float frac = gridStep;
                for (unsigned i = 1; i < gridMaxIdx; ++i, frac += gridStep)
                {
                    gridPoints[i][0] = gridPoints[0][0] * (1.f - frac) + gridPoints[gridMaxIdx][0] * frac;
                    gridPoints[i][gridMaxIdx] = gridPoints[0][gridMaxIdx] * (1.f - frac) + gridPoints[gridMaxIdx][gridMaxIdx] * frac;
                }
                
                for (unsigned i = 0; i < gridSize; ++i)
                {
                    frac = gridStep;
                    for (unsigned j = 1; j < gridMaxIdx; ++j, frac += gridStep)
                        gridPoints[i][j] = gridPoints[i][0] * (1.f - frac) + gridPoints[i][gridMaxIdx] * frac;
                }

                auto & out_points = pointsPerFrame[frame].first;
                auto & out_ids = pointsPerFrame[frame].second;

                for (unsigned i = 0; i < gridSize; ++i)
                {
                    for (unsigned j = 0; j < gridSize; ++j)
                    {
                        if (gridPoints[i][j].x > 0.f && gridPoints[i][j].x < 1.f &&
                            gridPoints[i][j].y > 0.f && gridPoints[i][j].y < 1.f)
                        {
                            out_ids.push_back( get_point_id(layerName, i, j) );
                            out_points.push_back( cv::KeyPoint(gridPoints[i][j].x * w, gridPoints[i][j].y * h, 100.f) );
                        }
                    }
                }
                
                frameNode = frameNode->NextSiblingElement("Frame");
            }
        }
        file = readdir(dir);
    }

    closedir(dir);
}

// Reads tracked mesh vertices from Mocha export file (Nuke nk format) and stores with IDs for matching across frames
void mesh_points(const std::string& dirpath, std::map<int, stella_vslam_bfx::prematched_points>& pointsPerFrame, int w, int h)
{
    auto dir = opendir(dirpath.c_str());
    struct dirent * file = readdir(dir);
    std::string line, layerName;
    std::map<unsigned, std::map<unsigned, std::pair<float, float>>> trackedPointPositions; // maps point ID from file to per-frame positions
    const float max_y = float(h);
    unsigned frame;
    while (file != nullptr)
    {
        if (file->d_type == DT_REG)
        {
            std::ifstream data(dirpath + "/" + file->d_name);
            if (data)
            {
                while (std::getline(data, line))
                {
                    if (line.length())
                    {
                        if (line.rfind("name", 0) == 0) // Layer name - comes after all the points
                        {
                            layerName = line.substr(5, line.length());

                            // Transfer collected points to output
                            for (const auto& pointPosns : trackedPointPositions)
                            {
                                auto id = get_point_id(layerName, pointPosns.first, 1000);
                                for (const auto& pos : pointPosns.second)
                                {
                                    auto p = pos.second;
                                    if (p.first > 0.f && p.first < w && p.second > 0.f && p.second < h)
                                    {
                                        frame = pos.first;
                                        pointsPerFrame[frame].first.push_back( cv::KeyPoint(p.first, p.second, 100.f) );
                                        pointsPerFrame[frame].second.push_back(id);
                                    }
                                }
                            }

                            // Clear stores for next layer
                            trackedPointPositions.clear();
                        }
                        else if (line.rfind("{ {curve K x1 1} ", 0) == 0)   // positions for a point
                        {
                            auto firstQuotePos = line.find_first_of('"');
                            auto secondQuotePos = line.find_first_of('"', firstQuotePos + 1);
                            unsigned fileID = std::stoi( line.substr(firstQuotePos + 1, secondQuotePos - firstQuotePos - 1) );
                            auto & pointPosns = trackedPointPositions[fileID];

                            // x values
                            auto curveStart = line.find_first_of('{', secondQuotePos),
                                curveEnd = line.find_first_of('}', curveStart + 1),
                                xPos = line.find_first_of('x', curveStart + 1);
                            while (xPos < curveEnd)
                            {
                                auto spacePos1 = line.find_first_of(' ', xPos + 1), spacePos2 = line.find_first_of(' ', spacePos1 + 1);
                                frame = std::stoi( line.substr(xPos + 1, spacePos1 - xPos - 1) );
                                pointPosns[frame].first = std::stof( line.substr(spacePos1 + 1, spacePos2 - spacePos1 - 1) );

                                xPos = line.find_first_of('x', xPos + 1); 
                            }

                            // y values
                            curveStart = line.find_first_of('{', curveEnd + 1);
                            curveEnd = line.find_first_of('}', curveStart + 1);
                            xPos = line.find_first_of('x', curveStart + 1);
                            while (xPos < curveEnd)
                            {
                                auto spacePos1 = line.find_first_of(' ', xPos + 1), spacePos2 = line.find_first_of(' ', spacePos1 + 1);
                                frame = std::stoi( line.substr(xPos + 1, spacePos1 - xPos - 1) );
                                pointPosns[frame].second = max_y - std::stof( line.substr(spacePos1 + 1, spacePos2 - spacePos1 - 1) );

                                xPos = line.find_first_of('x', xPos + 1);
                            }
                        }
                    }
                }
            }
        }
        file = readdir(dir);
    }
    closedir(dir);
}

std::tuple<std::shared_ptr<stella_vslam::data::keyframe>, int> earliest_valid_keyframe(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms,
                                                                                 std::map<double, int> const& timestampToVideoFrame)
{
    if (!keyfrms.empty()) {
        std::shared_ptr<stella_vslam::data::keyframe> first_keyframe_data = *std::min_element(keyfrms.begin(), keyfrms.end(),
                                                                                        [](const auto& a, const auto& b) { // earliest above landmark threshold
                                                                                                  int landmarkThreshold(10);
                                                                                                  bool validLandmarksA(a->get_valid_landmarks().size()>=landmarkThreshold);
                                                                                                  bool validLandmarksB(b->get_valid_landmarks().size()>=landmarkThreshold);
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

void printKeyframeLandmarks(std::vector<std::shared_ptr<stella_vslam::data::keyframe>> const& keyfrms,
                            std::map<double, int> const& timestampToVideoFrame) {
    spdlog::info("Map keyframe:-");
    for (auto const& keyfrm : keyfrms) {
        if (!keyfrm)
            spdlog::info("  [deleted]");
        else if (keyfrm->will_be_erased())
            spdlog::info("  [will be deleted]");
        else {
            int numValidLandmarks = keyfrm->get_valid_landmarks().size();
            const unsigned int min_num_obs_thr(1);
            int numTrackedLandmarks = keyfrm->get_num_tracked_landmarks(min_num_obs_thr);
            int numFeatures = keyfrm->frm_obs_.num_keypts_;

            auto f = timestampToVideoFrame.find(keyfrm->timestamp_);
            int videoFrame = (f != timestampToVideoFrame.end()) ? f->second : -1;

            spdlog::info("  id {}, timecode {}, vif frm {}, valid landmarks {}, tracked {}, features {}",
                         keyfrm->id_, keyfrm->timestamp_, videoFrame,
                         numValidLandmarks, numTrackedLandmarks, numFeatures);
        }
    }
}

void mono_tracking(const std::shared_ptr<stella_vslam::system>& slam,
                   const std::shared_ptr<stella_vslam::config>& cfg,
                   const std::string& video_file_path,
                   const std::string& mask_img_path,
                   const unsigned int frame_skip,
                   const unsigned int start_time,
                   const bool no_sleep,
                   const bool wait_loop_ba,
                   const bool auto_term,
                   const std::string& eval_log_dir,
                   const std::string& map_db_path,
                   const double start_timestamp,
				       const std::string& planar_path, const std::string& mesh_path, 
                   unsigned gridSize, YAML::Node yaml_node)
{
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    double initialFocalLength = (cfg->settings_.camera_model_ == stella_vslam::camera::model_type_t::Perspective) ? cfg->settings_.perspective_settings_.fx_ : 0;

    std::map<int, Eigen::Matrix4d> videoFrameToCamera;

    // Create a functor object for creating evaluation videos
    std::map<double, int> timestampToVideoFrame;
    slam->camera_->autocalibration_parameters_.writeMapVideo = [&video_file_path, &timestampToVideoFrame](stella_vslam::data::map_database const* map, std::string const& filename) {
      return stella_vslam_bfx::create_evaluation_video(video_file_path, filename, map, timestampToVideoFrame, nullptr);
    };

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(
        stella_vslam::util::yaml_optional_ref(yaml_node, "PangolinViewer"), slam, slam->get_frame_publisher(), slam->get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(
        stella_vslam::util::yaml_optional_ref(yaml_node, "SocketPublisher"), slam, slam->.get_frame_publisher(), slam->get_map_publisher());
#endif

    auto video = cv::VideoCapture(video_file_path, cv::CAP_FFMPEG);
    if (!video.isOpened()) {
        std::cerr << "Unable to open the video." << std::endl;
        return;
    }
    video.set(0, start_time);
    std::vector<double> track_times;

    std::map<int, stella_vslam_bfx::prematched_points> extraPointsPerFrame;
    if (!planar_path.empty())
        planar_points(planar_path, extraPointsPerFrame, video.get(cv::CAP_PROP_FRAME_WIDTH), video.get(cv::CAP_PROP_FRAME_HEIGHT), gridSize);
    if (!mesh_path.empty())
        mesh_points(mesh_path, extraPointsPerFrame, video.get(cv::CAP_PROP_FRAME_WIDTH), video.get(cv::CAP_PROP_FRAME_HEIGHT));
    const bool useExtraPoints = !extraPointsPerFrame.empty();

    cv::Mat frame;

    unsigned int num_frame = 0;
    double timestamp = start_timestamp;

    bool is_not_end = true;
    // run the slam in another thread
    std::thread thread([&]() {
        while (is_not_end) {

            // wait until the loop BA is finished
            if (wait_loop_ba) {
                while (slam->loop_BA_is_running() || !slam->mapping_module_is_enabled()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }

            is_not_end = video.read(frame);

            spdlog::info("Read video frame {})", num_frame);

            //bfxTestDrawFrame(frame);

            timestampToVideoFrame[timestamp] = num_frame;

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!frame.empty() && (num_frame % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                auto extraPoints = useExtraPoints && extraPointsPerFrame.find(num_frame) != extraPointsPerFrame.end() && !extraPointsPerFrame.at(num_frame).first.empty()
                                       ? &extraPointsPerFrame.at(num_frame)
                                       : nullptr;
                slam->feed_monocular_frame(frame, timestamp, mask, extraPoints);

            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (num_frame % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep) {
                const auto wait_time = 1.0 / slam->get_camera()->fps_ - track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            timestamp += 1.0 / slam->get_camera()->fps_;
            ++num_frame;

            // check if the termination of slam system is requested or not
            if (slam->terminate_is_requested()) {
                break;
            }
        }

        // Map from video frame -> timestamp
        std::vector<double> videoFrameToTimestamp(num_frame);
        for (auto const& tf : timestampToVideoFrame)
            videoFrameToTimestamp[tf.second] = tf.first;

        // Track (with mapping) backwards from the first keyframe
        auto [first_keyframe_data, first_keyframe] = earliest_valid_keyframe(slam->map_db_->get_all_keyframes(), timestampToVideoFrame);
        if (first_keyframe_data) {
            bool resultRelocalise = slam->relocalize_by_pose(first_keyframe_data->get_pose_wc());
            if (resultRelocalise==false)
                spdlog::warn("Failed to relocalise to first keyframe pose (video frame {})", first_keyframe);
        }
        spdlog::info("Relocalised to first keyframe pose (video frame {})", first_keyframe);

        printKeyframeLandmarks(slam->map_db_->get_all_keyframes(), timestampToVideoFrame);

        //int first_keyframe(-1); // First find the earliest keyframe
        //auto keyfrms = slam->map_db_->get_all_keyframes();
        //if (!keyfrms.empty()) {

        //   auto [first_keyframe_data, first_keyframe] = earliest_valid_keyframe(slam->map_db_->get_all_keyframes(), timestampToVideoFrame);



        //    std::shared_ptr<stella_vslam::data::keyframe> first_keyframe_data = *std::min_element(keyfrms.begin(), keyfrms.end(),
        //                                                                                          [](const auto& a, const auto& b) { return a->timestamp_ < b->timestamp_; });
        //    auto f = timestampToVideoFrame.find(first_keyframe_data->timestamp_);
        //    if (f != timestampToVideoFrame.end())
        //        first_keyframe = f->second;

        //    bool ok = slam->relocalize_by_pose(first_keyframe_data->get_pose_wc());
        //}
        int frame_count(num_frame);

        if (first_keyframe > 0) {
            for (num_frame = first_keyframe; num_frame >= 0; --num_frame) {
                spdlog::info("Seeking to frame  {}", num_frame);

                bool ok = video.set(cv::CAP_PROP_POS_FRAMES, num_frame);
                if (!ok)
                    spdlog::warn("Failed to seek to video frame {}", num_frame);

                bool ok2 = video.read(frame);
                if (!ok2)
                    spdlog::warn("Failed to read video frame  {}", num_frame);

                timestamp = videoFrameToTimestamp[num_frame];

                const auto tp_1 = std::chrono::steady_clock::now();

                if (!frame.empty() && (num_frame % frame_skip == 0)) {
                    // input the current frame and estimate the camera pose
                    auto extraPoints = useExtraPoints && extraPointsPerFrame.find(num_frame) != extraPointsPerFrame.end() && !extraPointsPerFrame.at(num_frame).first.empty()
                                           ? &extraPointsPerFrame.at(num_frame)
                                           : nullptr;
                    auto cameraPosePtr = slam->feed_monocular_frame(frame, timestamp, mask, extraPoints);
                    if (cameraPosePtr)
                        videoFrameToCamera[num_frame] = *cameraPosePtr;
                }

                const auto tp_2 = std::chrono::steady_clock::now();

                const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
                if (num_frame % frame_skip == 0) {
                    track_times.push_back(track_time);
                }

                // wait until the timestamp of the next frame
                //if (!no_sleep) {
                //    const auto wait_time = 1.0 / cfg->camera_->fps_ - track_time;
                //    if (0.0 < wait_time) {
                //        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                //    }
                //}

                // check if the termination of SLAM system is requested or not
                if (slam->terminate_is_requested()) {
                    break;
                }

                if (num_frame == 0) // the for loop doesn't work because num_frame is unsigned
                    break;
            }
        }

        auto [first_keyframe_data_new, first_keyframe_new] = earliest_valid_keyframe(slam->map_db_->get_all_keyframes(), timestampToVideoFrame);
        spdlog::info("Moved earliest keyframe from {} to {}", first_keyframe, first_keyframe_new);

        // Final bundle
        if (true) {
            slam->run_loop_BA();
            std::this_thread::sleep_for(std::chrono::microseconds(1000000)); // required?
        }

        spdlog::info("run_video_slam here 1 ");
        // wait until the loop BA is finished
        while (slam->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
        spdlog::info("run_video_slam here 2 ");

        // Second pass without mapping to fill in non-keyframes
        slam->disable_mapping_module();
        video.set(cv::CAP_PROP_POS_FRAMES, 0);
        is_not_end = true;
        num_frame = 0;
        while (is_not_end) {
            
            is_not_end = video.read(frame);
            
            spdlog::info("is-not_end {}", is_not_end);

            timestamp = videoFrameToTimestamp[num_frame];

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!frame.empty() && (num_frame % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                auto extraPoints = useExtraPoints && extraPointsPerFrame.find(num_frame) != extraPointsPerFrame.end() && !extraPointsPerFrame.at(num_frame).first.empty()
                                       ? &extraPointsPerFrame.at(num_frame)
                                       : nullptr;
                auto cameraPosePtr = slam->feed_monocular_frame(frame, timestamp, mask, extraPoints);
                if (cameraPosePtr)
                    videoFrameToCamera[num_frame] = *cameraPosePtr;
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (num_frame % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            //if (!no_sleep) {
            //    const auto wait_time = 1.0 / cfg->camera_->fps_ - track_time;
            //    if (0.0 < wait_time) {
            //        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
            //    }
            //}

            ++num_frame;

            // check if the termination of SLAM system is requested or not
            if (slam->terminate_is_requested()) {
                break;
            }
        }





        stella_vslam::data::map_database* map_db = slam->map_db_;
        bool ok = stella_vslam_bfx::create_evaluation_video(video_file_path, std::to_string((int)initialFocalLength), map_db, timestampToVideoFrame, &videoFrameToCamera);
        //bool ok = slam->camera_->autocalibration_parameters_.writeMapVideo(map_db, std::to_string((int)initialFocalLength));

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the slam process
    slam->shutdown();

    if (!eval_log_dir.empty()) {
        // output the trajectories for evaluation
        slam->save_frame_trajectory(eval_log_dir + "/frame_trajectory.txt", "TUM");
        slam->save_keyframe_trajectory(eval_log_dir + "/keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs(eval_log_dir + "/track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        slam->save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    backward::SignalHandling sh;
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto video_file_path = op.add<popl::Value<std::string>>("m", "video", "video file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto start_time = op.add<popl::Value<unsigned int>>("s", "start-time", "time to start playing [milli seconds]", 0);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto wait_loop_ba = op.add<popl::Switch>("", "wait-loop-ba", "wait until the loop BA is finished");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto log_level = op.add<popl::Value<std::string>>("", "log-level", "log level", "info");
    auto eval_log_dir = op.add<popl::Value<std::string>>("", "eval-log-dir", "store trajectory and tracking times at this path (Specify the directory where it exists.)", "");
    auto map_db_path_in = op.add<popl::Value<std::string>>("i", "map-db-in", "load a map from this path", "");
    auto map_db_path_out = op.add<popl::Value<std::string>>("o", "map-db-out", "store a map database at this path after slam", "");
    auto disable_mapping = op.add<popl::Switch>("", "disable-mapping", "disable mapping");
    auto start_timestamp = op.add<popl::Value<double>>("t", "start-timestamp", "timestamp of the start of the video capture", 0.0);
    auto planar_file_path = op.add<popl::Value<std::string>>("l", "planar", "planar tracks file path", "");
    auto mesh_file_path = op.add<popl::Value<std::string>>("", "mesh", "mesh tracks file path", "");
    auto grid_size = op.add<popl::Value<unsigned int>>("g", "grid-size", "grid size for extending planar points", 5);
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!op.unknown_options().empty()) {
        for (const auto& unknown_option : op.unknown_options()) {
            std::cerr << "unknown_options: " << unknown_option << std::endl;
        }
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !video_file_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    spdlog::set_level(spdlog::level::from_str(log_level->value()));

    // load configuration    
    YAML::Node yaml_node;
    try {
        //cfg = std::make_shared<stella_vslam::config>(config_file_path->value());
        yaml_node = YAML::LoadFile(config_file_path->value());
        
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Read settings from YAML
    auto settings = settings_from_yaml(yaml_node);
    std::shared_ptr<stella_vslam::config> cfg = std::make_shared<stella_vslam::config>(*settings);

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam->prof");
#endif

    // You cannot get timestamps of images with this input format.
    // It is recommended to specify the timestamp when the video recording was started in Unix time.
    // If not specified, the current system time is used instead.
    double timestamp = 0.0;
    if (!start_timestamp->is_set()) {
        //std::cerr << "--start-timestamp is not set. using system timestamp." << std::endl;
        //if (no_sleep->is_set()) {
        //    std::cerr << "If --no-sleep is set without --start-timestamp, timestamps may overlap between multiple runs." << std::endl;
        //}
        //std::chrono::system_clock::time_point start_time_system = std::chrono::system_clock::now();
        //timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(start_time_system.time_since_epoch()).count();
    }
    else {
        timestamp = start_timestamp->value();
    }

    // build a slam system
    auto slam = std::make_shared<stella_vslam::system>(cfg, vocab_file_path->value());
    bool need_initialize = true;
    if (map_db_path_in->is_set()) {
        need_initialize = false;
        //const auto path = fs::path(map_db_path_in->value());
        //if (path.extension() == ".yaml") {
        //    YAML::Node node = YAML::LoadFile(path);
        //    for (const auto& map_path : node["maps"].as<std::vector<std::string>>()) {
        //        slam->load_map_database(path.parent_path() / map_path);
        //    }
        //}
        //else {
        //    // load the prebuilt map
        //    slam->load_map_database(path);
        //}
        slam->load_map_database(map_db_path_in->value());
    }
    slam->startup(need_initialize);
    if (disable_mapping->is_set()) {
        slam->disable_mapping_module();
    }

    // run tracking
    if (slam->get_camera()->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        mono_tracking(slam,
                      cfg,
                      video_file_path->value(),
                      mask_img_path->value(),
                      frame_skip->value(),
                      start_time->value(),
                      no_sleep->is_set(),
                      wait_loop_ba->is_set(),
                      auto_term->is_set(),
                      eval_log_dir->value(),
                      map_db_path_out->value(),
                      timestamp,
					  planar_file_path->value(), mesh_file_path->value(),
                      grid_size->value(), yaml_node);

    // // run tracking
    // if (cfg->camera_->setup_type_ == stella_vslam::camera::setup_type_t::Monocular) {
        // mono_tracking(cfg, vocab_file_path->value(), video_file_path->value(), mask_img_path->value(),
                      // frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
                      // eval_log->is_set(), map_db_path_out->value(),
                      // planar_file_path->value(), mesh_file_path->value(),
                      // grid_size->value(), yaml_node);
					  
					  
					  
    }
    else {
        throw std::runtime_error("Invalid setup type: " + slam->get_camera()->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    if (settings)
        delete settings;

    return EXIT_SUCCESS;
}
