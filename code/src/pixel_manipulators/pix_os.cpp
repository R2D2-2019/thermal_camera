#include <pixel_manipulators/pix_os.hpp>

namespace r2d2::thermal_camera {
    pix_os_c::pix_os_c(mlx_parameters_s &params,
                       std::array<std::array<float, 32>, 24> &pixels,
                       kta_c &kta, kv_c &kv,
                       pix_os_ref_c &pix_os_ref)
        : pixel_manipulator_c(params, pixels),
          kta(kta),
          kv(kv),
          pix_os_ref(pix_os_ref) {
    }

    void pix_os_c::calculate_pixel(unsigned int row, unsigned int col) {
        pixels[row - 1][col - 1] =
            pixels[row - 1][col - 1] -
            pix_os_ref.get_value(row, col) *
                (1 + kta.get_value(row, col) * (params.Ta - params.TA0)) *
                (1 + kv.get_value(row, col) * (params.Vdd - params.VDD0));
    }
} // namespace r2d2::thermal_camera
