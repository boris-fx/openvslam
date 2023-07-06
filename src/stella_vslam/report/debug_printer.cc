#include "debug_printer.h"

#include "stella_vslam/type.h"
#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/landmark.h"

using namespace stella_vslam_bfx;

int debug_printer::precision = 3;
Eigen::IOFormat debug_printer::eigen_format =
    Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

template<typename T>
void debug_printer::print_keyframes(const T& keyframes, spdlog::level::level_enum log_level) {
    if (!spdlog::default_logger_raw()->should_log(log_level))
        return;

    std::stringstream msg_str;
    msg_str.precision(precision);

    msg_str << keyframes.size() << " keyframes:\n";
    for (auto keyfrm : keyframes) {
        auto camPose = keyfrm->get_pose_cw();
        msg_str << std::fixed << "\tKeyframe " << keyfrm->id_ << " camera pose: "
                << camPose.format(eigen_format) << std::endl;
    }

    spdlog::log(log_level, "{}", msg_str.str());
}

template<typename T>
void debug_printer::print_landmarks(const T& landmarks, spdlog::level::level_enum log_level) {
    if (!spdlog::default_logger_raw()->should_log(log_level))
        return;
    
    std::stringstream msg_str;
    msg_str.precision(precision);

    msg_str << landmarks.size() << " landmarks:\n";
    for (auto lm : landmarks) {
        if (!lm || lm->will_be_erased()) continue;
        auto pos = lm->get_pos_in_world();
        msg_str << std::fixed << "\t" << lm->id_ << " " << pos.format(eigen_format) << std::endl;
    }

    spdlog::log(log_level, "{}", msg_str.str());
}

template void debug_printer::print_keyframes(const std::vector<std::shared_ptr<stella_vslam::data::keyframe>>&, spdlog::level::level_enum);
template void debug_printer::print_keyframes(const stella_vslam::id_ordered_set<std::shared_ptr<stella_vslam::data::keyframe>>&, spdlog::level::level_enum);
template void debug_printer::print_keyframes(const std::unordered_set<std::shared_ptr<stella_vslam::data::keyframe>>&, spdlog::level::level_enum);

template void debug_printer::print_landmarks(const std::vector<std::shared_ptr<stella_vslam::data::landmark>>&, spdlog::level::level_enum);
template void debug_printer::print_landmarks(const stella_vslam::id_ordered_set<std::shared_ptr<stella_vslam::data::landmark>>&, spdlog::level::level_enum);
template void debug_printer::print_landmarks(const std::unordered_set<std::shared_ptr<stella_vslam::data::landmark>>&, spdlog::level::level_enum);
