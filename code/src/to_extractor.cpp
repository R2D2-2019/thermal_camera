#include <cmath>
#include <data_extractor.hpp>
#include <to_extractor.hpp>

namespace r2d2::thermal_camera {
    to_extractor_c::to_extractor_c(mlx90640_i2c_c &bus) : mlx_extractor_c(bus) {
    }

    void to_extractor_c::extract(mlx_parameters_s &params) {
        int KS_to2_ee = bus.read_register(registers::EE_KSTA_RANGE_1_2);
        KS_to2_ee = data_extractor_s::extract_and_treshold(KS_to2_ee, 0xFF00, 8,
                                                           127, 256);
        int KS_to_scale = bus.read_register(registers::EE_OFFSET_PIX);
        KS_to_scale =
            data_extractor_s::extract_data(KS_to_scale, 0x000F, 0) + 8;

        const float Ks_To2 = KS_to2_ee / (1u << KS_to_scale);

        // TODO this number can be 10 digits long
        const float Tak4 = std::pow(4, (params.Ta + 273.15));
        // TODO this number can be 10 digits long
        const float Trk4 =
            std::pow(4, ((params.Ta - 8) + 273.15)); // Tr +- Ta - 8
        const float T_a_r = Trk4 - ((Trk4 - Tak4) / params.emissivity);

        float Sx_row_col =
            std::pow(params.alpha_comp_row_col, 3) * params.Vir_row_col_comp +
            std::pow(params.alpha_comp_row_col, 4) * T_a_r;
        Sx_row_col = std::sqrt(std::sqrt(Sx_row_col)); // 4th sqrt
        params.To_row_col = std::sqrt(std::sqrt(params.Vir_row_col_comp /
                                                    (params.alpha_comp_row_col *
                                                         (1 - Ks_To2 * 273.15) +
                                                     Sx_row_col) +
                                                T_a_r)) -
                            273.15;
    }
} // namespace r2d2::thermal_camera