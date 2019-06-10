#include <cp_compensator.hpp>
#include <data_extractor.hpp>

namespace r2d2::thermal_camera {
    cp_compensator_c::cp_compensator_c(mlx90640_i2c_c &bus,
                                       const reading_pattern pattern)
        : mlx_extractor_c(bus), pattern(pattern) {
    }

    void cp_compensator_c::extract(mlx_parameters_s &params) {
        int data;

        data = bus.read_register(registers::EE_CP_OFF_DELTA_OFFSET_CP_SP0);
        const int off_cp_subpage_0 =
            data_extractor_s::extract_and_treshold(data, 0x03FF, 0, 511, 1024);
        const int off_cp_subpage_1_delta =
            data_extractor_s::extract_and_treshold(data, 0xFC00, 10, 31, 64);
        const int off_cp_subpage_1 = off_cp_subpage_0 + off_cp_subpage_1_delta;

        data = bus.read_register(registers::EE_CTRL_CALIB_KV_KTA_SCALE);
        const int Kta_scale_1 =
            data_extractor_s::extract_data(data, 0x00F0, 4) + 8;
        const int Kv_scale = data_extractor_s::extract_data(data, 0x0F00, 8);

        data = bus.read_register(registers::EE_KV_KTA_CP);
        const int Kta_cp_ee =
            data_extractor_s::extract_and_treshold(data, 0x00FF, 0, 127, 256);
        const int Kv_cp_ee =
            data_extractor_s::extract_and_treshold(data, 0xFF00, 8, 127, 256);
        const float Kv_cp = static_cast<float>(Kv_cp_ee) / (1u << Kv_scale);

        const float Kta_cp = // cast float, both are ints
            static_cast<float>(Kta_cp_ee) / (1u << Kta_scale_1);

        const float constant = (1 + Kta_cp * (params.Ta - params.TA0)) *
                               (1 + Kv_cp * (params.Vdd - params.VDD0));

        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            params.pix_os_cp_sp0 =
                params.pix_gain_cp_sp0 - off_cp_subpage_0 * constant;
            params.pix_os_cp_sp1 =
                params.pix_gain_cp_sp1 - off_cp_subpage_1 * constant;
            break;
        case reading_pattern::INTERLEAVED_MODE:
            data = bus.read_register(registers::EE_CHESS_CX);
            float IL_chess_c1 =
                static_cast<float>(data_extractor_s::extract_and_treshold(
                    data, 0x003F, 0, 31, 64));
            IL_chess_c1 /= 16.f;
            params.pix_os_cp_sp0 = params.pix_gain_cp_sp0 -
                                   (off_cp_subpage_0 + IL_chess_c1) * constant;
            params.pix_os_cp_sp1 = params.pix_gain_cp_sp1 -
                                   (off_cp_subpage_1 + IL_chess_c1) * constant;
            break;
        }
    }
} // namespace r2d2::thermal_camera
