#include <ee_kta_cp.hpp>

namespace r2d2::thermal_camera {
    ee_kta_cp_c::ee_kta_cp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_kta_cp_c::extract() {
        int data;
        
        data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        const int Kta_scale_1 =
            data_extractor::extract_data(data, 0x00F0, 4) + 8;
        
        data = bus.read_register(registers::EE_KV_KTA_CP);
        const int Kta_cp_ee =
            data_extractor::extract_and_treshold(data, 0x00FF, 0, 127, 256);
            
        params.Kta_cp = // cast float, both are ints
            static_cast<float>(Kta_cp_ee) / (1u << Kta_scale_1);
    }
} // namespace r2d2::thermal_camera
