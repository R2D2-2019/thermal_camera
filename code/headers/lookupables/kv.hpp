#pragma once

#include <lookupables/lookupable.hpp>

namespace r2d2::thermal_camera {

    class kv_c : public lookupable_c {
    private:
        int ee_kv_avg;
        int ee_ctrl_calib_kv_kta_scale;
    public:
        kv_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
} // namespace r2d2::thermal_camera
