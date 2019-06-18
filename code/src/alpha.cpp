#include <alpha.hpp>
#include <cmath>

namespace r2d2::thermal_camera {
    alpha_c::alpha_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : lookupable_c(bus, params) {
    }

    void alpha_c::calculate_pixel(unsigned int row, unsigned int col) {
        int data;
        // TODO: convert to appropriate row/col address.
        data = bus.read_register(registers::EE_ALPHA_ACC_SCALE);
        const int alpha_scale =
            data_extractor::extract_data(data, 0xF000, 12) + 30;
        const int ACC_scale_col = data_extractor::extract_data(data, 0x00F0, 4);
        const int ACC_scale_row = data_extractor::extract_data(data, 0x0F00, 8);
        const int ACC_scale_rem = data_extractor::extract_data(data, 0x000F, 0);

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

        data =
            bus.read_register(registers::EE_PIX_SENSITIVITY_AVG); // alpha ref

        // alpha(i, j) can get x E-07 or smaller
        table[row - 1]
             [col - 1] = /* in this calculation data equals alpha ref */
            (data + ACC_row * (1 << ACC_scale_row) +
             ACC_col * (1 << ACC_scale_col) +
             alpha_pixel_row_col * (1 << ACC_scale_rem)) /
            std::pow(
                2,
                alpha_scale); // Here we use std::pow because alpha_scale
                              // can get up to 38, which can cause an overflow
    }
} // namespace r2d2::thermal_camera