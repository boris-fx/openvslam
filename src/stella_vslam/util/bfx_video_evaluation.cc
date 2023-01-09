#include "bfx_video_evaluation.h"

#define USE_OPENCV_VIDEO_IO 0

#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>

#if USE_OPENCV_VIDEO_IO
#include <opencv2/videoio.hpp>
#endif

#include <stella_vslam/data/map_database.h>
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/util/bfx_video_evaluation.h>

namespace stella_vslam_bfx {

void MyEllipse(cv::Mat img, double angle) {
    float w = 400.0f;

    int thickness = 5;
    int lineType = 8;
    cv::ellipse(img,
                cv::Point(w / 2, w / 2),
                cv::Size(w / 4, w / 16),
                angle,
                0,
                360,
                cv::Scalar(255, 0, 0),
                thickness,
                lineType);
}

bool bfx_create_evaluation_video(std::string const& trackedVideoName, std::string const& testName,
                                 stella_vslam::data::map_database const* map_db, std::map<double, int> const& timestampToVideoFrame,
                                 std::map<int, Eigen::Matrix4d> const* videoFrameToCamera)
{
#if USE_OPENCV_VIDEO_IO

    int thickness = 2;
    int lineType = 8;
    int shift = 0; // Number of fractional bits in the coordinates of the center and in the radius value.

    using namespace std;
    using namespace cv;
    using namespace stella_vslam;

    bool askOutputType(false);

    if (!map_db)
        return false;

    auto keyfrms = map_db->get_all_keyframes();
    auto lms = map_db->get_all_landmarks();
    int numCameras = keyfrms.size();
    int numLocators = lms.size();

    std::map<int, std::shared_ptr<data::keyframe>> sourceFrameToKeyframe;

    camera::base* camera(nullptr);

    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm)
            continue;
        camera = keyfrm->camera_;
        auto f = timestampToVideoFrame.find(keyfrm->timestamp_);
        if (f != timestampToVideoFrame.end())
           sourceFrameToKeyframe[f->second] = keyfrm;
        //spdlog::info("keyframe id {} source id {} timestamp {}", keyfrm->id_, keyfrm->src_frm_id_, keyfrm->timestamp_);
    }

    spdlog::info("bfx_create_evaluation_video {} cameras, {} points", numCameras, numLocators);

    cv::VideoCapture inputVideo(trackedVideoName); // Open input
    if (!inputVideo.isOpened()) {
        spdlog::error("bfx_create_evaluation_video could not open the input video: {}", trackedVideoName);
        return false;
    }
    string::size_type pAt = trackedVideoName.find_last_of('.');                // Find extension point
    const string outputVideoName = trackedVideoName.substr(0, pAt) + "_" + testName + ".mp4"; // Form the new name with container
    int ex = static_cast<int>(inputVideo.get(CAP_PROP_FOURCC)); // Get Codec Type- Int form
    // Transform from int to char via Bitwise operators
    char EXT[] = {(char)(ex & 0XFF), (char)((ex & 0XFF00) >> 8), (char)((ex & 0XFF0000) >> 16), (char)((ex & 0XFF000000) >> 24), 0};
    Size S = Size((int)inputVideo.get(CAP_PROP_FRAME_WIDTH), // Acquire input size
                  (int)inputVideo.get(CAP_PROP_FRAME_HEIGHT));
    std::unique_ptr<VideoWriter> outputVideo = std::make_unique<VideoWriter>(); // Open the output
    if (askOutputType)
        outputVideo->open(outputVideoName, ex = -1, inputVideo.get(CAP_PROP_FPS), S, true);
    else
        outputVideo->open(outputVideoName, ex, inputVideo.get(CAP_PROP_FPS), S, true);
    if (!outputVideo->isOpened()) {
        spdlog::error("bfx_create_evaluation_video could not open the output video: {}", outputVideoName);
        return false;
    }
    cout << "Input frame resolution: Width=" << S.width << "  Height=" << S.height
         << " of nr#: " << inputVideo.get(CAP_PROP_FRAME_COUNT) << endl;
    cout << "Input codec type: " << EXT << endl;

    Mat src, res;
    vector<Mat> spl;
    int srcFrame(0);
    int cameraCount(0), noCameraCount(0);
    for (;;) //Show the image captured in the window and repeat
    {
        inputVideo >> src; // read

        if (src.empty())
            break; // check if at end

        Eigen::Matrix4d const* frameCamera(nullptr);
        if (videoFrameToCamera) {
            auto fCamera = videoFrameToCamera->find(srcFrame);
            if (fCamera != videoFrameToCamera->end())
                frameCamera = &fCamera->second;
        }
        if (frameCamera) {
            Vec2_t reproj;
            float x_right; // ???
            //using EigenTransform = Eigen::Transform<double, 3, Eigen::TransformTraits::AffineCompact>;
            using EigenTransform = Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;
            EigenTransform cameraToWorld(*frameCamera);
           // const Eigen::Matrix3d R(cameraToWorld.rotation().matrix());
            //const Eigen::Vector3d t(cameraToWorld.translation().data());
            
            EigenTransform worldToCamera = cameraToWorld.inverse();
            const Eigen::Matrix3d R(worldToCamera.rotation().matrix());
            const Eigen::Vector3d t(worldToCamera.translation().data());

            for (auto const& landmark : lms) {
                camera->reproject_to_image(R, t, landmark->get_pos_in_world(), reproj, x_right);
                cv::circle(src, cv::Point(reproj(0), reproj(1)), 2,
                           cv::Scalar(0, 255, 0), thickness,
                           lineType, shift);
            }
        }

        // Find associated keyframe data
        auto foundKeyframe = sourceFrameToKeyframe.find(srcFrame);
        std::shared_ptr<data::keyframe> keyframe;
        if (foundKeyframe != sourceFrameToKeyframe.end())
            keyframe = foundKeyframe->second;
        if (keyframe) {
            
            

            /*
            // Observations - red
            for (auto const& keypt : keyframe->frm_obs_.undist_keypts_) {
                            cv::circle(src, keypt.pt, 8,
                           cv::Scalar(255, 0, 0), thickness,
                           lineType, shift);
            }

            // Bearing vectors - pink
            std::vector<cv::Point2f> undist_pts_from_bearings;
            camera->convert_bearings_to_points(keyframe->frm_obs_.bearings_, undist_pts_from_bearings);
            for (auto const& point : undist_pts_from_bearings) {
                cv::circle(src, point, 6,
                           cv::Scalar(255, 0, 255), thickness,
                           lineType, shift);
            }
            */

            // Reprojections - green
            if (!frameCamera) { // generic camera takes precedence if there's a generic and keyframe camera
                Vec2_t reproj;
                float x_right; // ???
                for (auto const& landmark : lms) {
                    camera->reproject_to_image(keyframe->get_rot_cw(), keyframe->get_trans_cw(), landmark->get_pos_in_world(), reproj, x_right);
                    cv::circle(src, cv::Point(reproj(0), reproj(1)), 2,
                               cv::Scalar(0, 255, 0), thickness,
                               lineType, shift);
                }
            }
        }

        //    MyEllipse(src, 90);
        //    MyEllipse(src, 0);
        //    MyEllipse(src, 45);
        //    MyEllipse(src, -45);

        //float p = 400.0f;
        //    int markerType = cv::MARKER_CROSS;
        //    int markerSize = 20;
        //    cv::drawMarker(src, cv::Point(p / 2, p / 2), cv::Scalar(0, 255, 0),
        //                   markerType, markerSize, thickness,
        //                   lineType);

        //    int shift = 0; // Number of fractional bits in the coordinates of the center and in the radius value.
        //    cv::circle(src, cv::Point(500, 500), 5,
        //               cv::Scalar(0, 0, 255), thickness,
        //               lineType, shift);

        //    //cv::InputArrayOfArrays pts;
        //    //cv::fillPoly(img, pts,
        //    //             cv::Scalar(0, 0, 255), lineType, shift, cv::Point(0,0));

        //    std::vector<cv::Point> vPts = {
        //        {600, 600},
        //        {600, 610},
        //        {610, 610},
        //        {610, 600}};
        //    //const Point *pts, int npts,
        //    cv::fillConvexPoly(src, &vPts[0], vPts.size(),
        //                       cv::Scalar(0, 0, 255), lineType,
        //                       shift);

        if (keyframe || frameCamera) {
            ++cameraCount;
            *outputVideo << src;
        }
        else
            ++noCameraCount;

        ++srcFrame;
    }

    spdlog::info("bfx_create_evaluation_video writing video file, frames {}, missing {}, total {}", cameraCount, noCameraCount, srcFrame);
    outputVideo.reset();
    spdlog::info("bfx_create_evaluation_video wrote file {}", outputVideoName);

    return true;
#else
    return false;
#endif
}

} // namespace stella_vslam_bfx