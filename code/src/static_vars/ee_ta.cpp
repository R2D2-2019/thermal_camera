#include <static_vars/ee_ta.hpp>

namespace r2d2::thermal_camera {
    ee_ta_c::ee_ta_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_ta_c::extract() {
        int data;

        data = bus.read_register(registers::EE_KV_KT_PTAT);
        params.KVptat = static_cast<float>(
            data_extractor::extract_and_treshold(data, 0xFC00, 10, 31, 64));
        params.KVptat /= 4096;
        
        params.KTptat = static_cast<float>(
            data_extractor::extract_and_treshold(data, 0x03FF, 0, 511, 1024));
        params.KTptat /= 8;

        data = bus.read_register(registers::EE_PTAT25);
        params.Vptat25 = data_extractor::apply_treshold(data);

        data = bus.read_register(registers::EE_SCALE_OCC);
        params.alpha_ptat = static_cast<float>(
            data_extractor::extract_data(data, 0xF000, 12));
        params.alpha_ptat = (params.alpha_ptat / 4) + 8;
    }
} // namespace r2d2::thermal_camera