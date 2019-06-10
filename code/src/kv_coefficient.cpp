#include <data_extractor.hpp>
#include <kv_coefficient.hpp>

namespace r2d2::thermal_camera {
    kv_coefficient_c::kv_coefficient_c(mlx90640_i2c_c &bus, int row, int col)
        : mlx_extractor_c(bus), row(row), col(col) {
        offset_addr = registers::EE_OFFSET_PIX + get_pixel_number(row, col);
    }

    void kv_coefficient_c::extract(mlx_parameters_s &params) {
        int data;

        data = bus.read_register(offset_addr);
        const int Kta_ee =
            data_extractor_s::extract_and_treshold(data, 0x000E, 1, 3, 8);

        const uint8_t row_even = !(row % 2);
        const uint8_t col_even = !(col % 2);
        const uint8_t row_odd = !row_even;
        const uint8_t col_odd = !row_odd;

        // either 0x2436 or 0x2437
        const uint16_t Kta_rc_ee_addr = registers::EE_KTA_AVG + col_even;
        // take msb or lsb
        const uint16_t Kta_rc_ee_mask = 0xFF00 >> (8 * row_even);
        /* from Kta_rc_ee_addr, & it with Kta_rc_ee_mask, shift it with either 8
         or 0 spots*/

        data = bus.read_register(Kta_rc_ee_addr);
        const int Kta_rc_ee = data_extractor_s::extract_and_treshold(
            data, Kta_rc_ee_mask, 8 * row_odd, 127, 256);

        data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        const int Kta_scale_1 =
            data_extractor_s::extract_data(data, 0x00F0, 4) + 8;
        const int Kta_scale_2 = data_extractor_s::extract_data(data, 0x00F, 0);

        params.Kta_row_col =
            (Kta_rc_ee + Kta_ee * (1u << Kta_scale_2)) / (1u << Kta_scale_1);
        // either shifts it 12, 8, 4 or 0 times.
        const uint8_t shift = row_odd * 4 + col_odd * 8;
        // Results can be: 0xF000, 0x0F00, 0x00F0, 0x000F
        const uint16_t Kv_mask = 0x000F << shift;

        data = bus.read_register(registers::EE_KV_AVG);
        params.Kv =
            data_extractor_s::extract_and_treshold(data, Kv_mask, shift, 7, 16);

        data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        const int Kv_scale = data_extractor_s::extract_data(data, 0x0F00, 8);

        params.Kv /= (1u << Kv_scale);
    }
} // namespace r2d2::thermal_camera
