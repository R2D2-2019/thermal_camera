#pragma once
#include <static_var.hpp>

namespace r2d2::thermal_camera {
    class ee_vdd_c : public static_var_c {
    public:
        ee_vdd_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        void extract() override;
    };

} // namespace r2d2::thermal_camera
