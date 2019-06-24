#include <static_vars/ee_ksta.hpp>


namespace r2d2::thermal_camera {
    ee_ksta_c::ee_ksta_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_ksta_c::extract() {
        int data = bus.read_register(registers::EE_KSTA_TGC);
        int KstaEE =
            data_extractor::extract_and_treshold(data, 0xFC00, 10, 31, 64);
        params.KsTa = KstaEE / 8192.f;
    }
} // namespace r2d2::thermal_camera
