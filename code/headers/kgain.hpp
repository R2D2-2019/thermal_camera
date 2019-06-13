#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Sets the gain parameter. Please note that this value is updated every
     * frame and it is the same for all pixels including CP regardless the
     * subpage number
     */
    class kgain_c : public mlx_extractor_c {
    public:
        kgain_c(mlx90640_i2c_c &bus);
        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
