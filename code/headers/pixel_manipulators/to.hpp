#pragma once

#include <pixel_manipulators/pixel_manipulator.hpp>
#include <reading_patterns.hpp>
#include <lookupables/alpha.hpp>

namespace r2d2::thermal_camera {
    class to_c : public pixel_manipulator_c {
    private:
        reading_pattern pattern;
        alpha_c &alpha;

    public:
        to_c(mlx_parameters_s &params,
             std::array<std::array<float, 32>, 24> &pixels,
             const reading_pattern pattern, alpha_c &alpha);

        void calculate_pixel(unsigned int row, unsigned int col);
    };
} // namespace r2d2::thermal_camera
