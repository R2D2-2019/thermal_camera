#include <lookupable.hpp>

namespace r2d2::thermal_camera {
    lookupable_c::lookupable_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : bus(bus), params(params) {
    }

    int lookupable_c::get_pixel_number(unsigned int row, unsigned int col) const {
        // row * 32 =  row << 5
        return ((row - 1) << 5) + col;
    }

    float lookupable_c::get_value(unsigned int row, unsigned int col) {
        return table[row - 1][col - 1];
    }

} // namespace r2d2::thermal_camera