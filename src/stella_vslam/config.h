#ifndef STELLA_VSLAM_CONFIG_H
#define STELLA_VSLAM_CONFIG_H

#include "config_settings.h"
#include "stella_vslam/camera/base.h"
#include "stella_vslam/feature/orb_params.h"

namespace stella_vslam {

namespace marker_model {
class base;
}

class config {
public:
    //! Constructor
    explicit config(const stella_vslam_bfx::config_settings& settings);

    //! Destructor
    ~config();

    friend std::ostream& operator<<(std::ostream& os, const config& cfg);

    //! Settings/parameters
    const stella_vslam_bfx::config_settings& settings_;

    //! Camera model
    camera::base* camera_ = nullptr;

    //! ORB feature extraction model
    feature::orb_params* orb_params_ = nullptr;

    //! Marker model
    std::shared_ptr<marker_model::base> marker_model_ = nullptr;
};

} // namespace stella_vslam

#endif // STELLA_VSLAM_CONFIG_H
