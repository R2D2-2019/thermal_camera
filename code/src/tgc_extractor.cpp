#include <data_extractor.hpp>
#include <tgc_extractor.hpp>

namespace r2d2::thermal_camera {
    tgc_extractor_c::tgc_extractor_c(mlx90640_i2c_c &bus)
        : mlx_extractor_c(bus) {
    }

    void tgc_extractor_c::extract(mlx_parameters_s &params) {
        int data = bus.read_register(registers::EE_KSTA_TGC);
        params.TGC = static_cast<float>(
            data_extractor_s::extract_and_treshold(data, 0x00FF, 0, 127, 256));
        params.TGC /= 32;
    }
} // namespace r2d2::thermal_camera
