#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus)
        : bus(bus), Kvdd(0), Vdd25(0) {
        set_Kgain();
    }

    void mlx90640_processor_c::set_and_read_raw_pixels() {
        // TODO: To be implemented yet
    }

    uint8_t mlx90640_processor_c::get_resolution_correlation() const {
        uint16_t resolution_ee = extract_data(EE_RESOLUTION, 0x3000, 12);
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

    void mlx90640_processor_c::check_within_limits(uint8_t &row,
                                                   uint8_t &col) const {
        if (row > 24) {
            row = 24;
        }
        if (col > 32) {
            col = 32;
        } else if (row == 0) {
            row = 1;
        } else if (col == 0) {
            col = 1;
        }
    }

    float mlx90640_processor_c::get_Vdd() {
        Kvdd = get_compensated_data(EE_VDD_PIX, 0xFF00, 8, 127, 256);
        Kvdd *= 32;

        Vdd25 = extract_data(EE_VDD_PIX, 0x00FF, 0);
        Vdd25 = (Vdd25 - 256) * 32 - 8192;

        int ram_vdd_pix = read_and_apply_treshold(RAM_VDD_PIX);

        uint8_t res_cor = get_resolution_correlation();
        float Vdd = ((res_cor * ram_vdd_pix - Vdd25) / Kvdd) + VDD0;
        return Vdd;
    }

    float mlx90640_processor_c::get_Ta() const {
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

        int Vptat25 = read_and_apply_treshold(EE_PTAT25);
        int Vptat = read_and_apply_treshold(RAM_TA_PTAT);
        int Vbe = read_and_apply_treshold(RAM_TA_VBE);

        int alpha_ptat = extract_data(EE_SCALE_OCC, 0xF000, 12);
        alpha_ptat = (alpha_ptat / 4) + 8;

        float Vptat_art = (Vptat / (Vptat * alpha_ptat + Vbe)) * 262144;
        float Ta =
            (((Vptat_art / (1 + KVptat * delta_V)) - Vptat25) / KTptat) + 25;

        return Ta;
    }

    void mlx90640_processor_c::set_Kgain() {
        int gain = read_and_apply_treshold(EE_GAIN);
        int ram_gain = read_and_apply_treshold(RAM_GAIN);
        Kgain = static_cast<float>(gain) / ram_gain;
    }

    float mlx90640_processor_c::apply_pix_gain(uint8_t row, uint8_t col) const {
        check_within_limits(row, col);
        uint16_t addr = row * 32 + col + RAM_PAGE_START;
        int data = read_and_apply_treshold(addr);
        return data * Kgain; // = pix_gain
    }

    int mlx90640_processor_c::get_offset_calculation(uint8_t row,
                                                     uint8_t col) const {
        check_within_limits(row, col);
        int offset_average = read_and_apply_treshold(EE_PIX_OS_AVERAGE);
        int offset_row_col = EE_OFFSET_PIX + ((row - 1) * 32) + col;
        offset_row_col =
            get_compensated_data(offset_row_col, 0xFC00, 10, 31, 64);

        uint16_t row_addr = EE_OCC_ROWS_START + (row - 1) / 4;
        uint16_t col_addr = EE_OCC_COLS_START + (col - 1) / 4;
        uint16_t row_mask = 0x0F << 4 * ((row - 1) % 4);
        uint16_t col_mask = 0x0F << 4 * ((col - 1) % 4);

        int Occ_row_x = get_compensated_data(row_addr, row_mask,
                                             4 * ((row - 1) % 4), 7, 16);
        int Occ_col_x = get_compensated_data(col_addr, col_mask,
                                             4 * ((col - 1) % 4), 7, 16);

        int Occ_scale_row = extract_data(EE_SCALE_OCC, 0x0F00, 8);
        int Occ_scale_col = extract_data(EE_SCALE_OCC, 0x00F0, 4);
        int Occ_scale_rem = extract_data(EE_SCALE_OCC, 0x000F, 0);

        int Pix_OS_ref = offset_average +
                         Occ_row_x * std::pow(2, Occ_scale_row) +
                         Occ_col_x * std::pow(2, Occ_scale_col) +
                         offset_row_col * std::pow(2, Occ_scale_rem);
        return Pix_OS_ref;
    }

} // namespace r2d2::thermal_camera