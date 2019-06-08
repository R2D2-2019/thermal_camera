#include <cmath>
#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus,
                                               float emissivity)
        : bus(bus),
          Kvdd(0),
          Vdd25(0),
          KstaEE(0),
          Vdd(0.f),
          Kgain(0.f),
          Kta_row_col(0.f),
          Ta(0.f),
          pix_gain_cp_sp0(0.f),
          pix_gain_cp_sp1(0.f),
          pix_os_cp_sp0(0.f),
          pix_os_cp_sp1(0.f),
          TGC(0.f),
          emissivity(emissivity) {
        set_Kgain();
        set_Ta();
        set_Vdd();
        set_cp_gain();
        set_TGC();
        set_Ksta_EE();
    }

    uint8_t mlx90640_processor_c::get_resolution_correlation() const {
        uint16_t resolution_ee =
            extract_data(registers::EE_CTRL_CALIB_KV_KTA_SCALE, 0x3000, 12);
        uint16_t resolution_reg =
            extract_data(registers::INTERNAL_CONTROL_REGISTER, 0xFFFF, 10);
        resolution_ee = (1u << resolution_ee);
        resolution_reg = (1u << resolution_reg);
        return static_cast<uint8_t>(resolution_ee / resolution_reg);
    }

    uint16_t mlx90640_processor_c::extract_data(const uint16_t reg_addr,
                                                const uint16_t and_bits,
                                                const uint8_t shifted) const {
        return (bus.read_register(reg_addr) & and_bits) >> shifted;
    }

    void mlx90640_processor_c::apply_treshold(int &value,
                                              const uint16_t exceeds,
                                              const int minus) const {
        if (value > exceeds) {
            value -= minus;
        }
    }

    int mlx90640_processor_c::get_compensated_data(const uint16_t reg_addr,
                                                   const uint16_t and_bits,
                                                   const uint8_t shifted,
                                                   const uint16_t exceeds,
                                                   const int minus) const {
        int data = extract_data(reg_addr, and_bits, shifted);
        apply_treshold(data, exceeds, minus);
        return data;
    }

    int mlx90640_processor_c::read_and_apply_treshold(
        const uint16_t reg_addr) const {
        int data = bus.read_register(reg_addr);
        apply_treshold(data, 32767, 65536);
        return data;
    }

    void mlx90640_processor_c::check_within_limits(int &row, int &col) const {
        if (row > 24) {
            row = 24;
        } else if (row == 0) {
            row = 1;
        }
        if (col > 32) {
            col = 32;
        } else if (col == 0) {
            col = 1;
        }
    }

    void mlx90640_processor_c::set_Vdd() {
        Kvdd = get_compensated_data(registers::EE_VDD_PIX, 0xFF00, 8, 127, 256);
        Kvdd *= 32;

        Vdd25 = extract_data(registers::EE_VDD_PIX, 0x00FF, 0);
        Vdd25 = (Vdd25 - 256) * 32 - 8192;

        const int ram_vdd_pix = read_and_apply_treshold(registers::RAM_VDD_PIX);

        const uint8_t res_cor = get_resolution_correlation();
        Vdd = ((res_cor * ram_vdd_pix - Vdd25) / Kvdd) + VDD0;
    }

    void mlx90640_processor_c::set_Ta() {
        int data;

        data =
            get_compensated_data(registers::EE_KV_KT_PTAT, 0xFC00, 10, 31, 64);
        float KVptat = static_cast<float>(data);
        KVptat /= 4096;

        data = get_compensated_data(registers::EE_KV_KT_PTAT, 0x03FF, 0, 511,
                                    2024);
        float KTptat = static_cast<float>(data);
        KTptat /= 8;

        data = read_and_apply_treshold(registers::RAM_VDD_PIX);
        float delta_V = static_cast<float>(data);
        delta_V = (delta_V - Vdd25) / Kvdd;

        const int Vptat25 = read_and_apply_treshold(registers::EE_PTAT25);
        const int Vptat = read_and_apply_treshold(registers::RAM_TA_PTAT);
        const int Vbe = read_and_apply_treshold(registers::RAM_TA_VBE);

        int alpha_ptat = extract_data(registers::EE_SCALE_OCC, 0xF000, 12);
        alpha_ptat = (alpha_ptat / 4) + 8;

        const float Vptat_art = (Vptat / (Vptat * alpha_ptat + Vbe)) * 262144;
        Ta = (((Vptat_art / (1 + KVptat * delta_V)) - Vptat25) / KTptat) + TA0;
    }

    void mlx90640_processor_c::set_Kgain() {
        const int gain = read_and_apply_treshold(registers::EE_GAIN);
        const int ram_gain = read_and_apply_treshold(registers::RAM_GAIN);
        Kgain = static_cast<float>(gain) / ram_gain;
    }

    float mlx90640_processor_c::get_Kv_coefficient(int row, int col,
                                                   uint16_t offset_addr) {
        const int Kta_ee = get_compensated_data(offset_addr, 0x000E, 1, 3, 8);

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
        const int Kta_rc_ee = get_compensated_data(
            Kta_rc_ee_addr, Kta_rc_ee_mask, 8 * row_odd, 127, 256);

        const int Kta_scale_1 =
            extract_data(registers::EE_CTRL_CALIB_KV_KTA_SCALE, 0x00F0, 4) + 8;
        const int Kta_scale_2 =
            extract_data(registers::EE_CTRL_CALIB_KV_KTA_SCALE, 0x000F, 0);

        Kta_row_col =
            (Kta_rc_ee + Kta_ee * (1u << Kta_scale_2)) / (1u << Kta_scale_1);
        // either shifts it 12, 8, 4 or 0 times.
        const uint8_t shift = row_odd * 4 + col_odd * 8;
        // Results can be: 0xF000, 0x0F00, 0x00F0, 0x000F
        const uint16_t Kv_mask = 0x000F << shift;
        float Kv =
            get_compensated_data(registers::EE_KV_AVG, Kv_mask, shift, 7, 16);
        const int Kv_scale =
            extract_data(registers::EE_CTRL_CALIB_KV_KTA_SCALE, 0x0F00, 8);
        Kv /= (1u << Kv_scale);
        return Kv;
    }

    float mlx90640_processor_c::get_pix_gain(int row, int col) const {
        const uint16_t addr =
            ((row - 1) * 32 + (col - 1)) + registers::RAM_PAGE_START;
        const int data = read_and_apply_treshold(addr);
        return data * Kgain; // = pix_gain, returns a float since Kgain is float
    }

    void mlx90640_processor_c::set_cp_gain() {
        int data;

        data = read_and_apply_treshold(registers::RAM_CP_SP0);
        pix_gain_cp_sp0 = data * Kgain;
        // Kgain is floating point
        data = read_and_apply_treshold(registers::RAM_CP_SP1);
        pix_gain_cp_sp1 = data * Kgain;
    }

    int mlx90640_processor_c::get_patron(int row, int col) const {
        int pixel_number = get_pixel_number(row, col);
        int patron =
            (static_cast<int>((pixel_number - 1) / 32) -
             static_cast<int>(static_cast<int>((pixel_number - 1) / 32) / 2) *
                 2);

        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            /* If we are in chess pattern mode we need to xor it. */
            patron = patron ^ ((pixel_number - 1) -
                               static_cast<int>((pixel_number - 1) / 2) * 2);
            break;
        case reading_pattern::INTERLEAVED_MODE:
            /* However, if we're in interleaved mode, we can leave patron as it
             * was */
            break;
        }
        return patron;
    }

    void mlx90640_processor_c::set_TGC() {
        TGC = static_cast<float>(
            get_compensated_data(registers::EE_KSTA_TGC, 0x00FF, 0, 127, 256));
        TGC /= 32;
    }

    void mlx90640_processor_c::compensate_cp() {
        const int off_cp_subpage_0 = get_compensated_data(
            registers::EE_CP_OFF_DELTA_OFFSET_CP_SP0, 0x03FF, 0, 511, 1024);
        const int off_cp_subpage_1_delta = get_compensated_data(
            registers::EE_CP_OFF_DELTA_OFFSET_CP_SP0, 0xFC00, 10, 31, 64);
        const int off_cp_subpage_1 = off_cp_subpage_0 + off_cp_subpage_1_delta;

        const int Kta_scale_1 =
            extract_data(registers::EE_CTRL_CALIB_KV_KTA_SCALE, 0x00F0, 4) + 8;
        const int Kta_cp_ee =
            get_compensated_data(registers::EE_KV_KTA_CP, 0x00FF, 0, 127, 256);

        const float Kta_cp = // cast float, both are ints
            static_cast<float>(Kta_cp_ee) / (1u << Kta_scale_1);

        const int Kv_scale =
            extract_data(registers::EE_CTRL_CALIB_KV_KTA_SCALE, 0x0F00, 8);
        const int Kv_cp_ee =
            get_compensated_data(registers::EE_KV_KTA_CP, 0xFF00, 8, 127, 256);
        const float Kv_cp = static_cast<float>(Kv_cp_ee) / (1u << Kv_scale);

        const float constant =
            (1 + Kta_cp * (Ta - TA0)) * (1 + Kv_cp * (Vdd - VDD0));

        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            pix_os_cp_sp0 = pix_gain_cp_sp0 - off_cp_subpage_0 * constant;
            pix_os_cp_sp1 = pix_gain_cp_sp1 - off_cp_subpage_1 * constant;
            break;
        case reading_pattern::INTERLEAVED_MODE:
            float IL_chess_c1 = static_cast<float>(get_compensated_data(
                registers::EE_CHESS_CX, 0x003F, 0, 31, 64));
            IL_chess_c1 /= 16.f;
            pix_os_cp_sp0 =
                pix_gain_cp_sp0 - (off_cp_subpage_0 + IL_chess_c1) * constant;
            pix_os_cp_sp1 =
                pix_gain_cp_sp1 - (off_cp_subpage_1 + IL_chess_c1) * constant;
            break;
        }
    }

    float mlx90640_processor_c::get_pix_OS(int row, int col) {
        const int offset_average =
            read_and_apply_treshold(registers::EE_PIX_OS_AVERAGE);
        /* converts row and col into an address*/
        const uint16_t offset_addr =
            registers::EE_OFFSET_PIX + get_pixel_number(row, col);
        const int offset_row_col =
            get_compensated_data(offset_addr, 0xFC00, 10, 31, 64);

        const uint16_t row_addr = registers::EE_OCC_ROWS_START + (row - 1) / 4;
        const uint16_t col_addr = registers::EE_OCC_COLS_START + (col - 1) / 4;
        const uint16_t row_mask = 0x0F << 4 * ((row - 1) % 4);
        const uint16_t col_mask = 0x0F << 4 * ((col - 1) % 4);

        const int Occ_row_x = get_compensated_data(row_addr, row_mask,
                                                   4 * ((row - 1) % 4), 7, 16);
        const int Occ_col_x = get_compensated_data(col_addr, col_mask,
                                                   4 * ((col - 1) % 4), 7, 16);

        const int Occ_scale = bus.read_register(registers::EE_SCALE_OCC);
        const int Occ_scale_row = (Occ_scale & 0x0F00) >> 8;
        const int Occ_scale_col = (Occ_scale & 0x00F0) >> 4;
        const int Occ_scale_rem = (Occ_scale & 0x000F);

        const int Pix_OS_ref = offset_average +
                               Occ_row_x * (1u << Occ_scale_row) +
                               Occ_col_x * (1u << Occ_scale_col) +
                               offset_row_col * (1u << Occ_scale_rem);

        /* Prevents function from being const*/
        const float Kv = get_Kv_coefficient(row, col, offset_addr);
        /* pix gain returns float */
        const float Pix_Os = get_pix_gain(row, col) -
                             Pix_OS_ref * (1 + Kta_row_col * (Ta - TA0)) *
                                 (1 + Kv * (Vdd - VDD0));
        return Pix_Os;
    }

    void mlx90640_processor_c::set_Ksta_EE() {
        KstaEE =
            get_compensated_data(registers::EE_KSTA_TGC, 0xFC00, 10, 31, 64);
    }

    int mlx90640_processor_c::get_pixel_number(int row, int col) const {
        return (row - 1) * 32 + col;
    }

    float mlx90640_processor_c::get_alpha_compensation(const int row,
                                                       const int col,
                                                       const float Pix_OS,
                                                       const int patron) const {
        const int ee_alpha_acc_scale_result =
            bus.read_register(registers::EE_ALPHA_ACC_SCALE);

        const int ee_cp_sp0_alpha =
            bus.read_register(registers::EE_CP_SP0_ALPHA);

        const int alpha_scale_cp =
            ((ee_alpha_acc_scale_result & 0xF000) >> 12) + 27;

        int CP_P1_P0_ratio = (ee_cp_sp0_alpha & 0xFC00) >> 10;
        apply_treshold(CP_P1_P0_ratio, 31, 64);

        const float alpha_cp_sp_0 =
            (ee_cp_sp0_alpha & 0x03FF) / (1u << alpha_scale_cp);
        const float alpha_cp_sp_1 =
            alpha_cp_sp_0 * (1 + (CP_P1_P0_ratio / 128));
        const float Ks_Ta = KstaEE / 8192;

        const int alpha_ref =
            bus.read_register(registers::EE_PIX_SENSITIVITY_AVG);
        const int alpha_scale =
            ((ee_alpha_acc_scale_result & 0xF000) >> 12) + 30;

        const uint8_t row_offset = registers::EE_ACC_COL + ((row - 1) / 4);
        const uint8_t col_offset = registers::EE_ACC_ROW + ((col - 1) / 4);
        const uint16_t row_mask = 0x0F << (4 * ((row - 1) % 4));
        const uint16_t col_mask = 0x0F << (4 * ((col - 1) % 4));

        const int ACC_row = get_compensated_data(row_offset, row_mask,
                                                 4 * ((row - 1) % 4), 7, 16);
        const int ACC_col = get_compensated_data(col_offset, col_mask,
                                                 4 * ((col - 1) % 4), 7, 16);

        const int ACC_scale_col = (ee_alpha_acc_scale_result & 0x00F0) >> 4;
        const int ACC_scale_row = (ee_alpha_acc_scale_result & 0x0F00) >> 8;
        const int ACC_scale_rem = ee_alpha_acc_scale_result & 0x000F;

        const int alpha_pixel_row_col = get_compensated_data(
            registers::EE_ACC_ROW_COL + get_pixel_number(row, col), 0x03F0, 4,
            31, 64);

        const float alpha_row_col =
            (alpha_ref + ACC_row * (1u << ACC_scale_row) +
             ACC_col * (1u << ACC_scale_col) +
             alpha_pixel_row_col * (1u << ACC_scale_rem)) /
            (1u << alpha_scale);
        const float alpha_comp_row_col =
            (alpha_row_col -
             TGC * ((1 - patron) * alpha_cp_sp_0 + patron * alpha_cp_sp_1)) *
            (1 + Ks_Ta * (Ta - TA0));

        return alpha_comp_row_col;
    }

    float
    mlx90640_processor_c::get_IR_gradient_compensation(const float Pix_Os,
                                                       const int patron) const {
        const float Vir_emissivity_comp = Pix_Os / emissivity;
        const float Vir_row_col_comp =
            Vir_emissivity_comp -
            TGC * ((1 - patron) * pix_os_cp_sp0 + patron * pix_os_cp_sp1);
        return Vir_row_col_comp;
    }

    float mlx90640_processor_c::get_offset_calculation(int row, int col) {
        check_within_limits(row, col);
        const float Pix_Os = get_pix_OS(row, col);
        compensate_cp();
        const int patron = get_patron(row, col);
        const float alpha_compensation =
            get_alpha_compensation(row, col, Pix_Os, patron);
        const float ir_gradient_compensation =
            get_IR_gradient_compensation(Pix_Os, patron);

        const int KS_to2_ee = get_compensated_data(registers::EE_KSTA_RANGE_1_2,
                                                   0xFF00, 8, 127, 256);
        const int KS_to_scale =
            extract_data(registers::EE_OFFSET_PIX, 0x000F, 0) + 8;
        const float Ks_To2 = KS_to2_ee / (1u << KS_to_scale);

        (void)Ks_To2;
        (void)alpha_compensation;
        (void)ir_gradient_compensation;
        return float{};
    }

    void
    mlx90640_processor_c::set_reading_pattern(const reading_pattern &pattern) {
        this->pattern = pattern;
    }
} // namespace r2d2::thermal_camera