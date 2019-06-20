#pragma once

#include <dynamic_vars/dynamic_var.hpp>
#include <reading_patterns.hpp>

namespace r2d2::thermal_camera {
    class off_ta_vdd_cp_c : public dynamic_var_c {
        reading_pattern pattern;
    public:
        off_ta_vdd_cp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params, const reading_pattern pattern);

        void re_calculate() override;
    };
} // namespace r2d2::thermal_camera
