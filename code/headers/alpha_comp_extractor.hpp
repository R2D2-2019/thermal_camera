#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Sets the alpha compensation (row, col)
     */
    class alpha_comp_extractor_c : public mlx_extractor_c {
    private:
        int row, col;

    public:
        alpha_comp_extractor_c(mlx90640_i2c_c &bus, int row, int col);

        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
