#pragma once

#include <lookupable.hpp>

namespace r2d2::thermal_camera {

    class kv_c : public lookupable_c {
    public:
        kv_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        void calculate_pixel(int row, int col) override;
    };
} // namespace r2d2::thermal_camera
