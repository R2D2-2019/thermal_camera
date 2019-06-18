#include <kv.hpp>

namespace r2d2::thermal_camera {
    kv_c::kv_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : lookupable_c(bus, params) {
    }

    void kv_c::calculate_pixel(unsigned int row, unsigned int col) {
        int data;

        const uint8_t row_odd = row % 2;
        const uint8_t col_odd = col % 2;

        // either shifts it 12, 8, 4 or 0 times.
        const uint8_t shift = row_odd * 4 + col_odd * 8;
        // Results can be: 0xF000, 0x0F00, 0x00F0, 0x000F
        const uint16_t Kv_mask = 0x000F << shift;

        data = bus.read_register(registers::EE_KV_AVG);
        float Kv_row_col = static_cast<float>(data_extractor::extract_and_treshold(
            data, Kv_mask, shift, 7, 16));

        data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        const int Kv_scale = data_extractor::extract_data(data, 0x0F00, 8);

        table[row - 1][col - 1] = Kv_row_col / (1u << Kv_scale);
    }
} // namespace r2d2::thermal_camera
