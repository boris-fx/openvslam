#include "stella_vslam/feature/orb_params.h"
#include "stella_vslam/config_settings.h"

#include <cmath>

#include <gtest/gtest.h>

using namespace stella_vslam;

TEST(orb_params, load_orb_settings_without_rectangle_mask) {

    stella_vslam_bfx::config_settings settings(stella_vslam::camera::model_type_t::Equirectangular,
                                            stella_vslam::camera::setup_type_t::Monocular,
                                            stella_vslam::camera::color_order_t::RGB,
                                            1024, 1024, 25.0);
    settings.scale_factor_ = 1.3;
    settings.num_levels_ = 12;
    settings.ini_fast_threshold_ = 25;
    settings.min_fast_threshold_ = 9;

    const auto params = feature::orb_params(settings);

    EXPECT_FLOAT_EQ(params.scale_factor_, 1.3);
    EXPECT_EQ(params.num_levels_, 12);
    EXPECT_EQ(params.ini_fast_thr_, 25);
    EXPECT_EQ(params.min_fast_thr_, 9);
}

TEST(orb_params, calc_scale_factors) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto scale_factors = feature::orb_params::calc_scale_factors(num_scale_levels, scale_factor);

    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(scale_factors.at(level), std::pow(scale_factor, level));
    }
}

TEST(orb_params, calc_inv_scale_factors) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto inv_scale_factors = feature::orb_params::calc_inv_scale_factors(num_scale_levels, scale_factor);

    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(inv_scale_factors.at(level), std::pow((1.0f / scale_factor), level));
    }
}

TEST(orb_params, calc_level_sigma_sq) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto level_sigma_sq = feature::orb_params::calc_level_sigma_sq(num_scale_levels, scale_factor);
    float scale_factor_at_level = 1.0;
    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(level_sigma_sq.at(level), scale_factor_at_level * scale_factor_at_level);
        scale_factor_at_level = scale_factor * scale_factor_at_level;
    }
}

TEST(orb_params, calc_inv_level_sigma_sq) {
    const unsigned int num_scale_levels = 10;
    const float scale_factor = 1.26;

    const auto inv_level_sigma_sq = feature::orb_params::calc_inv_level_sigma_sq(num_scale_levels, scale_factor);
    float scale_factor_at_level = 1.0;
    for (unsigned int level = 0; level < num_scale_levels; ++level) {
        EXPECT_FLOAT_EQ(inv_level_sigma_sq.at(level), 1.0f / (scale_factor_at_level * scale_factor_at_level));
        scale_factor_at_level = scale_factor * scale_factor_at_level;
    }
}
