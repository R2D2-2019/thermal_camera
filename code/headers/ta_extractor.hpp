#include <mlx_extractor.hpp>

#pragma once

namespace r2d2::thermal_camera {
    /**
     * Gets the ambient temperature of the pixels. Common for all pixels. This
     * is probably needed because the the pixels emit heat aswell, so that needs
     * to be compensated.
     */
    class ta_extractor_c : public mlx_extractor_c {
    public:
        ta_extractor_c(mlx90640_i2c_c &bus);
        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
