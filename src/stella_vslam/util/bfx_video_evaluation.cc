#include "bfx_video_evaluation.h"

#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <stella_vslam/data/map_database.h>
#include <stella_vslam/data/keyframe.h>
#include <stella_vslam/data/landmark.h>

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

bool bfx_create_evaluation_video(std::string const& trackedVideoName, std::string const& testName, stella_vslam::data::map_database* map_db) {
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

    for (const auto& keyfrm : keyfrms) {
        if (!keyfrm)
            continue;
        sourceFrameToKeyframe[keyfrm->src_frm_id_] = keyfrm;
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
        return -1;
    }
    cout << "Input frame resolution: Width=" << S.width << "  Height=" << S.height
         << " of nr#: " << inputVideo.get(CAP_PROP_FRAME_COUNT) << endl;
    cout << "Input codec type: " << EXT << endl;

    Mat src, res;
    vector<Mat> spl;
    int srcFrame(0);
    for (;;) //Show the image captured in the window and repeat
    {
#if 1
        inputVideo >> src; // read

        if (src.empty())
            break; // check if at end

        //split(src, spl); // process - extract only the correct channel
        //for (int i = 0; i < 3; ++i)
        //  if (i != channel)
        //    spl[i] = Mat::zeros(S, spl[0].type());
        //merge(spl, res);
        //outputVideo->write(res); //save or

        int thickness = 2;
        int lineType = 8;

        // Find associated keyframe data
        auto foundKeyframe = sourceFrameToKeyframe.find(srcFrame);
        std::shared_ptr<data::keyframe> keyframe;
        if (foundKeyframe != sourceFrameToKeyframe.end())
            keyframe = foundKeyframe->second;

        if (keyframe) {
            camera::base* camera = keyframe->camera_;
            Vec2_t reproj;
            float x_right; // ???
            for (auto const& landmark : lms) {
                camera->reproject_to_image(keyframe->get_rot_cw(), keyframe->get_trans_cw(), landmark->get_pos_in_world(), reproj, x_right);
                int shift = 0; // Number of fractional bits in the coordinates of the center and in the radius value.
                cv::circle(src, cv::Point(reproj(0), reproj(1)), 2,
                           cv::Scalar(0, 255, 0), thickness,
                           lineType, shift);
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

        if (keyframe)
            *outputVideo << src;

#else
        inputVideo >> src; // read
        if (src.empty())
            break;       // check if at end
        split(src, spl); // process - extract only the correct channel
        for (int i = 0; i < 3; ++i)
            if (i != channel)
                spl[i] = Mat::zeros(S, spl[0].type());
        merge(spl, res);
        //outputVideo->write(res); //save or
        outputVideo << res;
#endif

        ++srcFrame;
    }

    spdlog::info("bfx_create_evaluation_video writing video file...");
    outputVideo.reset();
    spdlog::info("bfx_create_evaluation_video wrote file {}", outputVideoName);

    return 0;
}

} // namespace stella_vslam_bfx