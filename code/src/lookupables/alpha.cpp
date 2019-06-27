#include <lookupables/alpha.hpp>
#include <cmath>

namespace r2d2::thermal_camera {
    alpha_c::alpha_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : lookupable_c(bus, params) {
        ee_alpha_acc_scale = bus.read_register(registers::EE_ALPHA_ACC_SCALE);
        alpha_ref = bus.read_register(registers::EE_PIX_SENSITIVITY_AVG);
    }

    void alpha_c::calculate_pixel(unsigned int row, unsigned int col) {
        int data;

        const int alpha_scale =
            data_extractor::extract_data(ee_alpha_acc_scale, 0xF000, 12) + 30;
        const int ACC_scale_col =
            data_extractor::extract_data(ee_alpha_acc_scale, 0x00F0, 4);
        const int ACC_scale_row =
            data_extractor::extract_data(ee_alpha_acc_scale, 0x0F00, 8);
        const int ACC_scale_rem =
            data_extractor::extract_data(ee_alpha_acc_scale, 0x000F, 0);

        /* calculating offset for correct addr & bit set */
        const uint16_t row_offset = registers::EE_ACC_COL + ((row - 1) / 4);
        const uint16_t col_offset = registers::EE_ACC_ROW + ((col - 1) / 4);
        const uint16_t row_mask = 0x0F << (4 * ((row - 1) % 4));
        const uint16_t col_mask = 0x0F << (4 * ((col - 1) % 4));

        data = bus.read_register(row_offset);
        const int ACC_row = data_extractor::extract_and_treshold(
            data, row_mask, 4 * ((row - 1) % 4), 7, 16);

        data = bus.read_register(col_offset);
        const int ACC_col = data_extractor::extract_and_treshold(
            data, col_mask, 4 * ((col - 1) % 4), 7, 16);

        data = bus.read_register(registers::EE_ACC_ROW_COL +
                                 get_pixel_number(row, col));
        const int alpha_pixel_row_col =
            data_extractor::extract_and_treshold(data, 0x03F0, 4, 31, 64);

        // alpha(i, j) can get E-07 or smaller
        table[row - 1][col - 1] =
            (alpha_ref + ACC_row * std::pow(2, ACC_scale_row) +
             ACC_col * std::pow(2, ACC_scale_col) +
             alpha_pixel_row_col * std::pow(2, ACC_scale_rem));

        table[row - 1][col - 1] /= std::pow(2, alpha_scale);
    }
} // namespace r2d2::thermal_camera