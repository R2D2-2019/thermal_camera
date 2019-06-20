#include <ee_gain.hpp>

namespace r2d2::thermal_camera {
    ee_gain_c::ee_gain_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_gain_c::extract() {
        int data = bus.read_register(registers::EE_GAIN);
        params.ee_gain = data_extractor::apply_treshold(data);
    }
} // namespace r2d2::thermal_camera
