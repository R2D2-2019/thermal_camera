#include <data_extractor.hpp>
#include <kgain.hpp>

namespace r2d2::thermal_camera {
    kgain_c::kgain_c(mlx90640_i2c_c &bus) : mlx_extractor_c(bus) {
    }

    void kgain_c::extract(mlx_parameters_s &params) {
        int data;

        data = bus.read_register(registers::EE_GAIN);
        const int gain = data_extractor_s::apply_treshold(data);

        data = bus.read_register(registers::RAM_GAIN);
        const int ram_gain = data_extractor_s::apply_treshold(data);
        
        params.Kgain = static_cast<float>(gain) / ram_gain;
    }
} // namespace r2d2::thermal_camera
