#include "stella_vslam/data/bfx_shared_camera_intrinsics.h"

#include <nlohmann/json.hpp>

namespace stella_vslam {
namespace data {

nlohmann::json bfx_shared_camera_intrinsics::to_json() const {
    return {{"focal_length_x_pixels", focal_length_x_pixels}};
}

} // namespace data
} // namespace stella_vslam
