#pragma once

#include <pixel_manipulators/pixel_manipulator.hpp>

namespace r2d2::thermal_camera {
    class gain_comp_c : public pixel_manipulator_c {
    private:
        mlx90640_i2c_c &bus;
    public:
        gain_comp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params,
                            std::array<std::array<float, 32>, 24> &pixels);
        
        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
} // namespace r2d2::thermal_camera
