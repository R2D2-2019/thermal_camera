#include <pix_os_ref.hpp>

namespace r2d2::thermal_camera {
    pix_os_ref_c::pix_os_ref_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : lookupable_c(bus, params) {
    }

    void pix_os_ref_c::calculate_pixel(unsigned int row, unsigned int col) {
        int data;

        data = bus.read_register(registers::EE_PIX_OS_AVERAGE);
        const int offset_average = data_extractor::apply_treshold(data);

        /* converts row and col into an address*/
        const uint16_t offset_addr =
            registers::EE_OFFSET_PIX + get_pixel_number(row, col);

        data = bus.read_register(offset_addr);
        const int offset_row_col =
            data_extractor::extract_and_treshold(data, 0xFC00, 10, 31, 64);

        /* register offset & byte shifting offset */
        const uint16_t row_addr = registers::EE_OCC_ROWS_START + (row - 1) / 4;
        const uint16_t col_addr = registers::EE_OCC_COLS_START + (col - 1) / 4;
        const uint16_t row_mask = 0x0F << 4 * ((row - 1) % 4);
        const uint16_t col_mask = 0x0F << 4 * ((col - 1) % 4);

        data = bus.read_register(row_addr);
        const int Occ_row_x = data_extractor::extract_and_treshold(
            data, row_mask, 4 * ((row - 1) % 4), 7, 16);

        data = bus.read_register(col_addr);
        const int Occ_col_x = data_extractor::extract_and_treshold(
            data, col_mask, 4 * ((col - 1) % 4), 7, 16);

        data = bus.read_register(registers::EE_SCALE_OCC);
        const int Occ_scale_row =
            data_extractor::extract_data(data, 0x0F00, 8);
        const int Occ_scale_col =
            data_extractor::extract_data(data, 0x00F0, 4);
        const int Occ_scale_rem =
            data_extractor::extract_data(data, 0x000F, 0);

        table[row - 1][col - 1] = offset_average +
                                  Occ_row_x * (1u << Occ_scale_row) +
                                  Occ_col_x * (1u << Occ_scale_col) +
                                  offset_row_col * (1u << Occ_scale_rem);
    }
} // namespace r2d2::thermal_camera
