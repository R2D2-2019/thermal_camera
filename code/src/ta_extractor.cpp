#include <data_extractor.hpp>
#include <ta_extractor.hpp>

namespace r2d2::thermal_camera {
    ta_extractor_c::ta_extractor_c(mlx90640_i2c_c &bus) : mlx_extractor_c(bus) {
    }

    void ta_extractor_c::extract(mlx_parameters_s &params) {
        int data;

        data = bus.read_register(registers::EE_KV_KT_PTAT);
        float KVptat = static_cast<float>(
            data_extractor_s::extract_and_treshold(data, 0xFC00, 10, 31, 64));
        KVptat /= 4096;

        float KTptat = static_cast<float>(
            data_extractor_s::extract_and_treshold(data, 0x03FF, 0, 511, 1024));
        KTptat /= 8;

        data = bus.read_register(registers::RAM_VDD_PIX);
        float delta_V =
            static_cast<float>(data_extractor_s::apply_treshold(data));
        delta_V = (delta_V - params.Vdd25) / params.Kvdd;

        data = bus.read_register(registers::EE_PTAT25);
        const int Vptat25 = data_extractor_s::apply_treshold(data);

        data = bus.read_register(registers::RAM_TA_PTAT);
        const int Vptat = data_extractor_s::apply_treshold(data);

        data = bus.read_register(registers::RAM_TA_VBE);
        const int Vbe = data_extractor_s::apply_treshold(data);

        data = bus.read_register(registers::EE_SCALE_OCC);
        float alpha_ptat = static_cast<float>(
            data_extractor_s::extract_data(data, 0xF000, 12));
        alpha_ptat = (alpha_ptat / 4) + 8;

        const float Vptat_art = (Vptat / (Vptat * alpha_ptat + Vbe)) * 262144;

        params.Ta =
            (((Vptat_art / (1 + KVptat * delta_V)) - Vptat25) / KTptat) +
            params.TA0;
    }
} // namespace r2d2::thermal_camera
