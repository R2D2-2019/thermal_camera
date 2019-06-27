#include <static_vars/ee_resolution.hpp>

namespace r2d2::thermal_camera {
    ee_resolution_c::ee_resolution_c(mlx90640_i2c_c &bus,
                                     mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_resolution_c::extract() {
        uint16_t data =
            bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        params.resolution_ee =
            data_extractor::extract_data(data, 0x3000, 12);
    }
} // namespace r2d2::thermal_camera
