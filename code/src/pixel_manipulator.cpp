#include <pixel_manipulator.hpp>

namespace r2d2::thermal_camera {
    pixel_manipulator_c::pixel_manipulator_c(
        mlx_parameters_s &params, std::array<std::array<float, 32>, 24> &pixels)
        : pixels(pixels), params(params) {
    }

    void pixel_manipulator_c::set_patron(unsigned int row, unsigned int col,
                                         const reading_pattern pattern) {
        int pixel_number = ((row - 1) << 5) + col;
        patron =
            (static_cast<int>((pixel_number - 1) / 32) -
             static_cast<int>(static_cast<int>((pixel_number - 1) / 32) / 2) *
                 2);

        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            /* If we are in chess pattern mode we need to xor it. */
            patron = patron ^ ((pixel_number - 1) -
                               static_cast<int>((pixel_number - 1) / 2) * 2);
            break;
        case reading_pattern::INTERLEAVED_MODE:
            /* However, if we're in interleaved mode, we can leave patron as it
             * was */
            break;
        }
    }
} // namespace r2d2::thermal_camera
