#include <data_extractor.hpp>
#include <pix_os.hpp>

namespace r2d2::thermal_camera {
    pix_os_c::pix_os_c(mlx90640_i2c_c &bus, int row, int col)
        : mlx_extractor_c(bus), row(row), col(col) {
    }

    float pix_os_c::get_pix_gain(const int row, const int col,
                                 mlx_parameters_s &params) const {
        const uint16_t addr =
            ((row - 1) * 32 + (col - 1)) + registers::RAM_PAGE_START;
        int data = bus.read_register(addr);
        data = data_extractor_s::apply_treshold(data);
        return data *
               params.Kgain; // = pix_gain, returns a float since Kgain is float
    }

    void pix_os_c::extract(mlx_parameters_s &params) {
        int data;

        data = bus.read_register(registers::EE_PIX_OS_AVERAGE);
        const int offset_average = data_extractor_s::apply_treshold(data);

        /* converts row and col into an address*/
        const uint16_t offset_addr =
            registers::EE_OFFSET_PIX + get_pixel_number(row, col);

        data = bus.read_register(offset_addr);
        const int offset_row_col =
            data_extractor_s::extract_and_treshold(data, 0xFC00, 10, 31, 64);

        /* register offset & byte shifting offset */
        const uint16_t row_addr = registers::EE_OCC_ROWS_START + (row - 1) / 4;
        const uint16_t col_addr = registers::EE_OCC_COLS_START + (col - 1) / 4;
        const uint16_t row_mask = 0x0F << 4 * ((row - 1) % 4);
        const uint16_t col_mask = 0x0F << 4 * ((col - 1) % 4);

        data = bus.read_register(row_addr);
        const int Occ_row_x = data_extractor_s::extract_and_treshold(
            data, row_mask, 4 * ((row - 1) % 4), 7, 16);

        data = bus.read_register(col_addr);
        const int Occ_col_x = data_extractor_s::extract_and_treshold(
            data, col_mask, 4 * ((col - 1) % 4), 7, 16);

        data = bus.read_register(registers::EE_SCALE_OCC);
        const int Occ_scale_row =
            data_extractor_s::extract_data(data, 0x0F00, 8);
        const int Occ_scale_col =
            data_extractor_s::extract_data(data, 0x00F0, 4);
        const int Occ_scale_rem =
            data_extractor_s::extract_data(data, 0x000F, 0);

        const int Pix_OS_ref = offset_average +
                               Occ_row_x * (1u << Occ_scale_row) +
                               Occ_col_x * (1u << Occ_scale_col) +
                               offset_row_col * (1u << Occ_scale_rem);

        /* pix gain returns float */
        params.Pix_Os =
            get_pix_gain(row, col, params) -
            Pix_OS_ref * (1 + params.Kta_row_col * (params.Ta - params.TA0)) *
                (1 + params.Kv * (params.Vdd - params.VDD0));
    }
} // namespace r2d2::thermal_camera
