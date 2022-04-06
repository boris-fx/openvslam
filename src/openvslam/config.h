#ifndef OPENVSLAM_CONFIG_H
#define OPENVSLAM_CONFIG_H

#include "config_settings.h"
#include "openvslam/camera/base.h"
#include "openvslam/feature/orb_params.h"

namespace openvslam {

class config {
public:
    //! Constructor
    explicit config(const openvslam_bfx::config_settings& settings);

    //! Destructor
    ~config();

    friend std::ostream& operator<<(std::ostream& os, const config& cfg);

    //! Settings/parameters
    const openvslam_bfx::config_settings& settings_;

    //! Camera model
    camera::base* camera_ = nullptr;

    //! ORB feature extraction model
    feature::orb_params* orb_params_ = nullptr;
};

} // namespace openvslam

#endif // OPENVSLAM_CONFIG_H
