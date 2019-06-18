#include <off_ta_vdd_cp.hpp>

namespace r2d2::thermal_camera {
    off_ta_vdd_cp_c::off_ta_vdd_cp_c(mlx90640_i2c_c &bus,
                                     mlx_parameters_s &params,
                                     const reading_pattern pattern)
        : dynamic_var_c(bus, params), pattern(pattern) {
    }

    void off_ta_vdd_cp_c::re_calculate() {
        float constant = (1 + params.Kta_cp * (params.Ta - params.TA0)) *
                         (1 + params.Kv_cp * (params.Vdd - params.VDD0));

        params.pix_os_cp_sp0 =
            params.pix_gain_cp_sp0 - params.off_cp_sp0 * constant;

        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            params.pix_os_cp_sp1 =
                params.pix_gain_cp_sp1 - params.off_cp_sp1 * constant;
            break;
        case reading_pattern::INTERLEAVED_MODE:
    	    int data = bus.read_register(registers::EE_CHESS_CX);
            float IL_chess_c1 =
                static_cast<float>(data_extractor::extract_and_treshold(
                    data, 0x003F, 0, 31, 64));
            IL_chess_c1 /= 16.f;
            params.pix_os_cp_sp1 = params.pix_gain_cp_sp1 -
                                   (params.off_cp_sp1 + IL_chess_c1) * constant;
            break;
        }
    }
} // namespace r2d2::thermal_camera
