#include <lookupables/kta.hpp>

namespace r2d2::thermal_camera {
    kta_c::kta_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : lookupable_c(bus, params) {
        int data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        Kta_scale_1 = data_extractor::extract_data(data, 0x00F0, 4) + 8;
        Kta_scale_2 = data_extractor::extract_data(data, 0x00F, 0);
    }

    void kta_c::calculate_pixel(unsigned int row, unsigned int col) {
        int data;

        uint16_t offset_addr =
            registers::EE_OFFSET_PIX + get_pixel_number(row, col);

        data = bus.read_register(offset_addr);
        const int Kta_ee =
            data_extractor::extract_and_treshold(data, 0x000E, 1, 3, 8);

        const uint8_t row_even = !(row % 2);
        const uint8_t col_even = !(col % 2);
        const uint8_t row_odd = !row_even;

        // either 0x2436 or 0x2437
        const uint16_t Kta_rc_ee_addr = registers::EE_KTA_AVG + col_even;
        // take msb or lsb
        const uint16_t Kta_rc_ee_mask = 0xFF00 >> (8 * row_even);

        /* from Kta_rc_ee_addr, & it with Kta_rc_ee_mask, shift it with either 8
         or 0 spots*/
        data = bus.read_register(Kta_rc_ee_addr);
        const int Kta_rc_ee = data_extractor::extract_and_treshold(
            data, Kta_rc_ee_mask, 8 * row_odd, 127, 256);

        // Here, 1 << x equals 2^x again
        table[row - 1][col - 1] = (Kta_rc_ee + Kta_ee * (1u << Kta_scale_2));
        table[row - 1][col - 1] /= (1ll << Kta_scale_1);
    }
} // namespace r2d2::thermal_camera