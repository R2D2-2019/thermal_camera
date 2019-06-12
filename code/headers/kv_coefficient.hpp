#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Sets the Kv coeffecient
     *
     */
    class kv_coefficient_c : public mlx_extractor_c {
    private:
        int row, col;
        uint16_t offset_addr;

    public:
        kv_coefficient_c(mlx90640_i2c_c &bus, int row, int col);

        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
