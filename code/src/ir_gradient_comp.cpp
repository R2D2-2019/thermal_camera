#include <ir_gradient_comp.hpp>

namespace r2d2::thermal_camera {
    ir_gradient_comp::ir_gradient_comp(
        mlx_parameters_s &params, std::array<std::array<float, 32>, 24> &pixels,
        const reading_pattern pattern)
        : pixel_manipulator_c(params, pixels), pattern(pattern) {
    }

    void ir_gradient_comp::calculate_pixel(int row, int col) {
        set_patron(row, col, pattern);
        pixels[row - 1][row - 1] =
            pixels[row - 1][col - 1] -
            params.TGC * ((1 - patron) * params.pix_os_cp_sp0 + patron *
                          params.pix_os_cp_sp1);
    }
} // namespace r2d2::thermal_camera
