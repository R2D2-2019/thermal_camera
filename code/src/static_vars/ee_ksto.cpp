#include <static_vars/ee_ksto.hpp>
#include <ostream.hpp>

namespace r2d2::thermal_camera {
    ee_ksto_c::ee_ksto_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_ksto_c::extract() {
        int data;

        data = bus.read_register(registers::EE_OFFSET_PIX);
        uint16_t ksto_scale = data_extractor::extract_data(data, 0x000F, 0) + 8;
        data = bus.read_register(registers::EE_KSTA_RANGE_1_2);
        int ksto1_ee = data_extractor::extract_and_treshold(data, 0x00FF, 0, 127, 256);
        params.ksto1 = static_cast<float>(ksto1_ee) / (1u << ksto_scale);

        int ksto2_ee = data_extractor::extract_and_treshold(data, 0xFF00, 8, 127, 256);
        params.ksto2 = static_cast<float>(ksto2_ee) / (1u << ksto_scale);

        data = bus.read_register(registers::EE_KSTA_RANGE_3_4);
        int ksto3_ee = data_extractor::extract_and_treshold(data, 0x00FF, 0, 127, 256);
        params.ksto3 = static_cast<float>(ksto3_ee) / (1u << ksto_scale);

        int ksto4_ee = data_extractor::extract_and_treshold(data, 0xFF00, 8, 127, 256);
        params.ksto4 = static_cast<float>(ksto4_ee) / (1u << ksto_scale);
    }
} // namespace r2d2::thermal_camera
