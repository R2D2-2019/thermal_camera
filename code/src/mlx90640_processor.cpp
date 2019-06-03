#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    mlx90640_processor_c::mlx90640_processor_c(mlx90640_i2c_c &bus)
        : bus(bus), Kvdd(0), Vdd25(0) {
    }

    void mlx90640_processor_c::set_and_read_raw_pixels() {
    }

    uint8_t mlx90640_processor_c::get_resolution_correlation() const {
        int16_t resolution_ee =
            static_cast<int16_t>(extract_data(EE_RESOLUTION, 0x3000, 12));
        int16_t resolution_reg = static_cast<int16_t>(
            extract_data(INTERNAL_CONTROL_REGISTER, 0xFFFF, 10));
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
                                              const uint16_t minus) const {
        if (value > exceeds) {
            value -= minus;
        }
    }

    int mlx90640_processor_c::get_compensated_data(const uint16_t reg_addr,
                                                   const uint16_t and_bits,
                                                   const uint8_t shifted,
                                                   const uint16_t exceeds,
                                                   const uint16_t minus) const {
        int data;
        data = extract_data(reg_addr, and_bits, shifted);
        apply_treshold(data, exceeds, minus);
        return data;
    }

    int16_t mlx90640_processor_c::get_Vdd() {
        Kvdd = get_compensated_data(EE_VDD_PIX, 0xFF00, 12, 127, 256);
        Kvdd *= 32;

        Vdd25 = extract_data(EE_VDD_PIX, 0x00FF, 0);
        Vdd25 = (Vdd25 - 256) * 32 - 8192;

        int ram_vdd_pix =
            get_compensated_data(RAM_VDD_PIX, 0xFFFF, 0, 32767, 65536);

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

        data = get_compensated_data(RAM_VDD_PIX, 0xFFFF, 0, 32767, 65536);
        float delta_V = static_cast<float>(data);
        delta_V = (delta_V - Vdd25) / Kvdd;

        int Vptat25 = bus.read_register(EE_PTAT25);
        apply_treshold(Vptat25, 32767, 65536);

        int Vptat = bus.read_register(RAM_TA_PTAT);
        apply_treshold(Vptat, 32767, 65536);

        int Vbe = bus.read_register(RAM_TA_VBE);
        apply_treshold(Vbe, 32767, 65536);

        int alpha_ptat_ee = extract_data(EE_SCALE_OCC, 0xF000, 12);

        int alpha_ptat = (alpha_ptat_ee / 4) + 8;

        float Vptat_art = (Vptat / (Vptat * alpha_ptat + Vbe)) * 262144;
        float Ta =
            (((Vptat_art / (1 + KVptat * delta_V)) - Vptat25) / KTptat) + 25;

        return Ta;
    }

    float mlx90640_processor_c::get_gain() const {
        int gain = bus.read_register(EE_GAIN);
        apply_treshold(gain, 32767, 65536);

        int ram_gain = bus.read_register(RAM_GAIN);
        apply_treshold(ram_gain, 32767, 65536);

        return static_cast<float>(gain / ram_gain);
    }

} // namespace r2d2::thermal_camera