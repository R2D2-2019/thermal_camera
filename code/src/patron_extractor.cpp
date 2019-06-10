#include <patron_extractor.hpp>

namespace r2d2::thermal_camera {
    patron_extractor_c::patron_extractor_c(mlx90640_i2c_c &bus, int row,
                                           int col,
                                           const reading_pattern pattern)
        : mlx_extractor_c(bus), row(row), col(col), pattern(pattern) {
    }

    void patron_extractor_c::extract(mlx_parameters_s &params) {
        int pixel_number = get_pixel_number(row, col);
        params.patron =
            (static_cast<int>((pixel_number - 1) / 32) -
             static_cast<int>(static_cast<int>((pixel_number - 1) / 32) / 2) *
                 2);

        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            /* If we are in chess pattern mode we need to xor it. */
            params.patron =
                params.patron ^ ((pixel_number - 1) -
                                 static_cast<int>((pixel_number - 1) / 2) * 2);
            break;
        case reading_pattern::INTERLEAVED_MODE:
            /* However, if we're in interleaved mode, we can leave patron as it
             * was */
            break;
        }
    }
} // namespace r2d2::thermal_camera
