#include <static_vars/ee_kv_cp.hpp>

namespace r2d2::thermal_camera {
    ee_kv_cp_c::ee_kv_cp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_kv_cp_c::extract() {
        int data;

        data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        const int Kv_scale = data_extractor::extract_data(data, 0x0F00, 8);

        data = bus.read_register(registers::EE_KV_KTA_CP);

        const int Kv_cp_ee =
            data_extractor::extract_and_treshold(data, 0xFF00, 8, 127, 256);
        // Here, 1 << x equals 2^x again
        params.Kv_cp = static_cast<float>(Kv_cp_ee) / (1u << Kv_scale);
    }
} // namespace r2d2::thermal_camera
