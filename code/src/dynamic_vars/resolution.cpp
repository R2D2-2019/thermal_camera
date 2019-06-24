#include <dynamic_vars/resolution.hpp>

namespace r2d2::thermal_camera {
    resolution_c::resolution_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : dynamic_var_c(bus, params) {
    }

    void resolution_c::re_calculate() {
        int data = bus.read_register(registers::INTERNAL_CONTROL_REGISTER);
        params.res_cor =
            (1u << params.resolution_ee) /
            (1u << data_extractor::extract_data(data, 0x0C00, 10));
    }
} // namespace r2d2::thermal_camera
