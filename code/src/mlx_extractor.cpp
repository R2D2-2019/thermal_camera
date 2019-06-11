#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    void print(const char *message, long long val) {
        hwlib::cout << message << ' ' << val << hwlib::endl;
    }

    mlx_extractor_c::mlx_extractor_c(mlx90640_i2c_c &bus) : bus(bus) {
    }

    int mlx_extractor_c::get_pixel_number(int row, int col) const {
        return (row - 1) * 32 + col;
    }
} // namespace r2d2::thermal_camera
