#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Sets the Vir row col compensation
     */
    class vir_extractor_c : public mlx_extractor_c {
    public:
        vir_extractor_c(mlx90640_i2c_c &bus);

        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera