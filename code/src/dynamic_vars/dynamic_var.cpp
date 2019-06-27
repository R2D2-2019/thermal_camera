#include <dynamic_vars/dynamic_var.hpp>

namespace r2d2::thermal_camera {
    dynamic_var_c::dynamic_var_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : bus(bus), params(params) {
    }
} // namespace r2d2::thermal_camera
