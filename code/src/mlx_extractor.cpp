#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    mlx_extractor_c::mlx_extractor_c(mlx90640_i2c_c &bus) : bus(bus) {
    }

    int mlx_extractor_c::get_pixel_number(int row, int col) const {
        // row * 32 =  row << 5
        return ((row - 1) << 5) + col;
    }
} // namespace r2d2::thermal_camera
