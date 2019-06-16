#include <alpha_comp_extractor.hpp>
#include <cmath>
#include <data_extractor.hpp>


namespace r2d2::thermal_camera {
    alpha_comp_extractor_c::alpha_comp_extractor_c(mlx90640_i2c_c &bus, int row,
                                                   int col)
        : mlx_extractor_c(bus), row(row), col(col) {
    }

    void alpha_comp_extractor_c::extract(mlx_parameters_s &params) {
        int data;

        data = bus.read_register(registers::EE_ALPHA_ACC_SCALE);
        const int alpha_scale =
            data_extractor_s::extract_data(data, 0xF000, 12) + 30;
        const int ACC_scale_col =
            data_extractor_s::extract_data(data, 0x00F0, 4);
        const int ACC_scale_row =
            data_extractor_s::extract_data(data, 0x0F00, 8);
        const int ACC_scale_rem =
            data_extractor_s::extract_data(data, 0x000F, 0);
        const int alpha_scale_cp =
            data_extractor_s::extract_data(data, 0xF000, 12) + 27;

        data = bus.read_register(registers::EE_CP_SP0_ALPHA);
        const int CP_P1_P0_ratio =
            data_extractor_s::extract_and_treshold(data, 0xFC00, 10, 31, 64);

        const float alpha_cp_sp_0 =
            data_extractor_s::extract_data(data, 0x03FF, 0) /
            std::pow(2, alpha_scale_cp); // use std pow, bitshifting can cause overflow

        const float alpha_cp_sp_1 =
            alpha_cp_sp_0 * (1 + (CP_P1_P0_ratio / 128));

        data = bus.read_register(registers::EE_KSTA_TGC);
        int KstaEE =
            data_extractor_s::extract_and_treshold(data, 0xFC00, 10, 31, 64);
        const float Ks_Ta = KstaEE / 8192.f;

        /* calculating offset for correct addr & bit set */
        const uint16_t row_offset = registers::EE_ACC_COL + ((row - 1) / 4);
        const uint16_t col_offset = registers::EE_ACC_ROW + ((col - 1) / 4);
        const uint16_t row_mask = 0x0F << (4 * ((row - 1) % 4));
        const uint16_t col_mask = 0x0F << (4 * ((col - 1) % 4));

        data = bus.read_register(row_offset);
        const int ACC_row = data_extractor_s::extract_and_treshold(
            data, row_mask, 4 * ((row - 1) % 4), 7, 16);

        data = bus.read_register(col_offset);
        const int ACC_col = data_extractor_s::extract_and_treshold(
            data, col_mask, 4 * ((col - 1) % 4), 7, 16);

        data = bus.read_register(registers::EE_ACC_ROW_COL +
                                 get_pixel_number(row, col));
        const int alpha_pixel_row_col =
            data_extractor_s::extract_and_treshold(data, 0x03F0, 4, 31, 64);

        data =
            bus.read_register(registers::EE_PIX_SENSITIVITY_AVG); // alpha ref
            
        // alpha(i, j) can get x E-07 or smaller
        const float
            alpha_row_col = /* in this calculation data equals alpha ref */
            (data + ACC_row * (1 << ACC_scale_row) +
             ACC_col * (1 << ACC_scale_col) +
             alpha_pixel_row_col * (1 << ACC_scale_rem)) /
            std::pow(2,
                     alpha_scale); // Here we use std::pow because alpha_scale
                                   // can get 38, which can cause an overflow
        params.alpha_comp_row_col =
            (alpha_row_col - params.TGC * ((1 - params.patron) * alpha_cp_sp_0 +
                                           params.patron * alpha_cp_sp_1)) *
            (1 + Ks_Ta * (params.Ta - params.TA0));
    }
} // namespace r2d2::thermal_camera