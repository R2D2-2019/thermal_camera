#include <static_vars/ee_tgc_extractor.hpp>

namespace r2d2::thermal_camera {
    ee_tgc_extractor_c::ee_tgc_extractor_c(mlx90640_i2c_c &bus,
                                           mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_tgc_extractor_c::extract() {
        int data = bus.read_register(registers::EE_KSTA_TGC);
        params.TGC = static_cast<float>(
            data_extractor::extract_and_treshold(data, 0x00FF, 0, 127, 256));
        params.TGC /= 32;
    }
} // namespace r2d2::thermal_camera
