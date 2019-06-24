#include <dynamic_vars/kgain.hpp>

namespace r2d2::thermal_camera {
    kgain_c::kgain_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : dynamic_var_c(bus, params) {
    }

    void kgain_c::re_calculate() {
        int ram_gain = bus.read_register(registers::RAM_GAIN);
        params.Kgain = static_cast<float>(ram_gain) / params.ee_gain;
    }
} // namespace r2d2::thermal_camera
