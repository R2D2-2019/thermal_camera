#include <ee_vdd.hpp>

namespace r2d2::thermal_camera {
    ee_vdd_c::ee_vdd_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_vdd_c::extract() {
        int data = bus.read_register(registers::EE_VDD_PIX);

        params.Kvdd =
            data_extractor_s::extract_and_treshold(data, 0xFF00, 8, 127, 256);
        // (Kvdd * 2^y) = (Kvdd << y)
        params.Kvdd <<= 5;

        params.Vdd25 = data_extractor_s::extract_data(data, 0x00FF, 0);
        // (x * 2^y) = (x << y)
        params.Vdd25 = ((params.Vdd25 - 256) << 5) - 8192;
    }

} // namespace r2d2::thermal_camera
