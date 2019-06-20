#pragma once

#include <pixel_manipulators/pixel_manipulator.hpp>

namespace r2d2::thermal_camera {
    class vir_compensator : public pixel_manipulator_c {
    public:
        vir_compensator(mlx_parameters_s &params,
                            std::array<std::array<float, 32>, 24> &pixels);

        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
} // namespace r2d2::thermal_camera
