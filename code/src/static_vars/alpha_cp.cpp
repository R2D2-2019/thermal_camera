#include <static_vars/alpha_cp.hpp>
#include <cmath>

namespace r2d2::thermal_camera {
    alpha_cp_c::alpha_cp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void alpha_cp_c::extract() {
        int data;

        data = bus.read_register(registers::EE_ALPHA_ACC_SCALE);

        const int alpha_scale_cp =
            data_extractor::extract_data(data, 0xF000, 12) + 27;

        data = bus.read_register(registers::EE_CP_SP0_ALPHA);
        const int CP_P1_P0_ratio =
            data_extractor::extract_and_treshold(data, 0xFC00, 10, 31, 64);

        params.alpha_cp_sp_0 =
            static_cast<float>(data_extractor::extract_data(data, 0x03FF, 0)) /
            std::pow(
                2,
                alpha_scale_cp); // use std pow, bitshifting can cause overflow

        params.alpha_cp_sp_1 =
            params.alpha_cp_sp_0 * (1.f + (CP_P1_P0_ratio / 128.f));
    }
} // namespace r2d2::thermal_camera
