#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Sets the TGC parameter
     */
    class tgc_extractor_c : public mlx_extractor_c {
    public:
        tgc_extractor_c(mlx90640_i2c_c &bus);

        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
