#pragma once

#include <pixel_manipulator.hpp>
#include <reading_patterns.hpp>

namespace r2d2::thermal_camera {
    class ir_gradient_comp : public pixel_manipulator_c {
        reading_pattern pattern;
        int patron;

    public:
        ir_gradient_comp(mlx_parameters_s &params,
                         std::array<std::array<float, 32>, 24> &pixels,
                         const reading_pattern pattern);

        void calculate_pixel(unsigned int row, unsigned int col);
    };
} // namespace r2d2::thermal_camera
