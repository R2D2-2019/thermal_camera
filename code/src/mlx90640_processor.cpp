#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus) : bus(bus) {
    }

    void mlx90640_processor_c::set_and_read_raw_pixels() {
    }

    int16_t mlx90640_processor_c::get_delta_V() {
        int16_t Kvdd = (bus.read_register(EE_VDD_PIX) & 0xFF00) / 256;
        if (Kvdd > 127) {
            Kvdd -= 256;
        }
        Kvdd *= 32;
        int16_t VDD25 = (bus.read_register(EE_VDD_PIX) - 256) * 32 - 8192;
        int16_t ram_vdd_pix = bus.read_register(RAM_VDD_PIX);
        if (ram_vdd_pix > 32767) {
            ram_vdd_pix -= 65536;
        }
        return static_cast<int16_t>((ram_vdd_pix - VDD25) / Kvdd);
    }

    int16_t mlx90640_processor_c::get_gain() {
        int16_t gain = bus.read_register(EE_GAIN);
        if (gain > 32767) {
            gain -= 65536;
        }
        int16_t ram_gain = bus.read_register(RAM_GAIN);
        if (ram_gain > 32767) {
            ram_gain -= 65536;
        }
        return static_cast<int16_t>(gain / ram_gain);
    }

} // namespace r2d2::thermal_camera