#include <cmath>
#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus)
        : bus(bus),
          Kvdd(0),
          Vdd25(0),
          Vdd(0.f),
          Kgain(0.f),
          Kta_row_col(0.f),
          Ta(0.f) {
        set_Kgain();
        set_Ta();
        set_Vdd();
    }

    uint8_t mlx90640_processor_c::get_resolution_correlation() const {
        uint16_t resolution_ee =
            extract_data(EE_CTRL_CALIB_KV_KTA_SCALE, 0x3000, 12);
        uint16_t resolution_reg =
            extract_data(INTERNAL_CONTROL_REGISTER, 0xFFFF, 10);
        resolution_ee = std::pow(2, resolution_ee);
        resolution_reg = std::pow(2, resolution_reg);
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

    int
    mlx90640_processor_c::read_and_apply_treshold(const uint16_t addr) const {
        int data = bus.read_register(addr);
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
        Kvdd = get_compensated_data(EE_VDD_PIX, 0xFF00, 8, 127, 256);
        Kvdd *= 32;

        Vdd25 = extract_data(EE_VDD_PIX, 0x00FF, 0);
        Vdd25 = (Vdd25 - 256) * 32 - 8192;

        const int ram_vdd_pix = read_and_apply_treshold(RAM_VDD_PIX);

        const uint8_t res_cor = get_resolution_correlation();
        Vdd = ((res_cor * ram_vdd_pix - Vdd25) / Kvdd) + VDD0;
    }

    void mlx90640_processor_c::set_Ta() {
        int data;

        data = get_compensated_data(EE_KV_KT_PTAT, 0xFC00, 10, 31, 64);
        float KVptat = static_cast<float>(data);
        KVptat /= 4096;

        data = get_compensated_data(EE_KV_KT_PTAT, 0x03FF, 0, 511, 2024);
        float KTptat = static_cast<float>(data);
        KTptat /= 8;

        data = read_and_apply_treshold(RAM_VDD_PIX);
        float delta_V = static_cast<float>(data);
        delta_V = (delta_V - Vdd25) / Kvdd;

        const int Vptat25 = read_and_apply_treshold(EE_PTAT25);
        const int Vptat = read_and_apply_treshold(RAM_TA_PTAT);
        const int Vbe = read_and_apply_treshold(RAM_TA_VBE);

        int alpha_ptat = extract_data(EE_SCALE_OCC, 0xF000, 12);
        alpha_ptat = (alpha_ptat / 4) + 8;

        const float Vptat_art = (Vptat / (Vptat * alpha_ptat + Vbe)) * 262144;
        Ta = (((Vptat_art / (1 + KVptat * delta_V)) - Vptat25) / KTptat) + TA0;
    }

    void mlx90640_processor_c::set_Kgain() {
        const int gain = read_and_apply_treshold(EE_GAIN);
        const int ram_gain = read_and_apply_treshold(RAM_GAIN);
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
        const uint16_t Kta_rc_ee_addr = EE_KTA_AVG + col_even;
        // take msb or lsb
        const uint16_t Kta_rc_ee_mask = 0xFF00 >> (8 * row_even);
        /* from Kta_rc_ee_addr, & it with Kta_rc_ee_mask, shift it with either 8
         or 0 spots*/
        const int Kta_rc_ee = get_compensated_data(
            Kta_rc_ee_addr, Kta_rc_ee_mask, 8 * row_odd, 127, 256);

        const int Kta_scale_1 =
            extract_data(EE_CTRL_CALIB_KV_KTA_SCALE, 0x00F0, 4) + 8;
        const int Kta_scale_2 =
            extract_data(EE_CTRL_CALIB_KV_KTA_SCALE, 0x000F, 0);

        Kta_row_col = (Kta_rc_ee + Kta_ee * std::pow(2, Kta_scale_2)) /
                      std::pow(2, Kta_scale_1);
        // either shifts it 12, 8, 4 or 0 times.
        const uint8_t shift = row_odd * 4 + col_odd * 8;
        // Results can be: 0xF000, 0x0F00, 0x00F0, 0x000F
        const uint16_t Kv_mask = 0x000F << shift;
        float Kv = get_compensated_data(EE_KVG_AVG, Kv_mask, shift, 7, 16);
        const int Kv_scale =
            extract_data(EE_CTRL_CALIB_KV_KTA_SCALE, 0x0F00, 8);
        Kv /= std::pow(2, Kv_scale);
        return Kv;
    }

    float mlx90640_processor_c::get_pix_gain(int row, int col) const {
        const uint16_t addr = ((row - 1) * 32 + (col - 1)) + RAM_PAGE_START;
        const int data = read_and_apply_treshold(addr);
        return data * Kgain; // = pix_gain, returns a float since Kgain is float
    }

    float mlx90640_processor_c::get_offset_calculation(int row, int col) {
        check_within_limits(row, col);
        const int offset_average = read_and_apply_treshold(EE_PIX_OS_AVERAGE);
        /* converts row and col into an address*/
        const uint16_t offset_addr = EE_OFFSET_PIX + ((row - 1) * 32) + col;
        const int offset_row_col =
            get_compensated_data(offset_addr, 0xFC00, 10, 31, 64);

        const uint16_t row_addr = EE_OCC_ROWS_START + (row - 1) / 4;
        const uint16_t col_addr = EE_OCC_COLS_START + (col - 1) / 4;
        const uint16_t row_mask = 0x0F << 4 * ((row - 1) % 4);
        const uint16_t col_mask = 0x0F << 4 * ((col - 1) % 4);

        const int Occ_row_x = get_compensated_data(row_addr, row_mask,
                                                   4 * ((row - 1) % 4), 7, 16);
        const int Occ_col_x = get_compensated_data(col_addr, col_mask,
                                                   4 * ((col - 1) % 4), 7, 16);

        const int Occ_scale_row = extract_data(EE_SCALE_OCC, 0x0F00, 8);
        const int Occ_scale_col = extract_data(EE_SCALE_OCC, 0x00F0, 4);
        const int Occ_scale_rem = extract_data(EE_SCALE_OCC, 0x000F, 0);

        const int Pix_OS_ref = offset_average +
                               Occ_row_x * std::pow(2, Occ_scale_row) +
                               Occ_col_x * std::pow(2, Occ_scale_col) +
                               offset_row_col * std::pow(2, Occ_scale_rem);

        /* Prevents function from being const*/
        const float Kv = get_Kv_coefficient(row, col, offset_addr);

        const float Pix_Os = get_pix_gain(row, col) -
                             Pix_OS_ref * (1 + Kta_row_col * (Ta - TA0)) *
                                 (1 + Kv * (Vdd - VDD0));
        return Pix_Os;
    }

} // namespace r2d2::thermal_camera