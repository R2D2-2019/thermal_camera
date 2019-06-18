#include <vir_compensator.hpp>

namespace r2d2::thermal_camera {
    vir_compensator::vir_compensator(
        mlx_parameters_s &params, std::array<std::array<float, 32>, 24> &pixels)
        : pixel_manipulator_c(params, pixels) {
    }

    void vir_compensator::calculate_pixel(int row, int col) {
        pixels[row - 1][col - 1] /= params.emissivity;
    }

} // namespace r2d2::thermal_camera
