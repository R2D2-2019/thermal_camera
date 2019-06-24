#include <cmath>
#include <pixel_manipulators/to.hpp>

namespace r2d2::thermal_camera {
    to_c::to_c(mlx_parameters_s &params,
               std::array<std::array<float, 32>, 24> &pixels,
               const reading_pattern pattern, lookupable_c &alpha)
        : pixel_manipulator_c(params, pixels), pattern(pattern), alpha(alpha) {
    }

    void to_c::calculate_pixel(unsigned int row, unsigned int col) {
        set_patron(row, col, pattern);
        // Datasheet section 11.2.2.8
        float alpha_comp_row_col =
            (alpha.get_value(row, col) -
             params.TGC * ((1 - patron) * params.alpha_cp_sp_0 +
                           patron * params.alpha_cp_sp_1)) *
            (1 + params.KsTa * (params.Ta - params.TA0));

        // Datasheet section 11.2.2.9
        /* Tak4, Trk4 and Ta_r can be 10 digits long. */
        const long double Tak4 = std::pow((params.Ta + 273.15), 4);
        const long double Trk4 =
            std::pow((params.Ta - 8) + 273.15, 4); // Tr +- Ta - 8
        const long double Ta_r = Trk4 - ((Trk4 - Tak4) / params.emissivity);

        float Sx_row_col =
            std::pow(alpha_comp_row_col, 3) * pixels[row - 1][col - 1] +
            std::pow(alpha_comp_row_col, 4) * Ta_r;

        Sx_row_col =
            params.ksto2 * std::sqrt(std::sqrt(Sx_row_col)); // 4th sqrt

        float To = std::sqrt(std::sqrt(
                       pixels[row - 1][col - 1] /
                           (alpha_comp_row_col * (1 - params.ksto2 * 273.15) +
                            Sx_row_col) +
                       Ta_r)) -
                   273.15;

        float ct, ksto, alpha_range;

        if (To < params.ct2) {
            ct = params.ct1;
            ksto = params.ksto1;
            alpha_range = params.alpha_corr_range1;
        } else if (To < params.ct3) {
            ct = params.ct2;
            ksto = params.ksto2;
            alpha_range = params.alpha_corr_range2;
        } else if (To < params.ct4) {
            ct = params.ct3;
            ksto = params.ksto3;
            alpha_range = params.alpha_corr_range3;
        } else {
            ct = params.ct4;
            ksto = params.ksto4;
            alpha_range = params.alpha_corr_range4;
        }
        // Datasheet section 11.2.2.9.1.3
        pixels[row - 1][col - 1] =
            std::sqrt(std::sqrt(pixels[row - 1][col - 1] /
                                    (alpha_comp_row_col * alpha_range *
                                     (1 + ksto * (To - ct))) +
                                Ta_r)) -
            273.15;
    }
} // namespace r2d2::thermal_camera
