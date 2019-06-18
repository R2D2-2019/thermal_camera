#pragma once

#include <array>
#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>
#include <reading_patterns.hpp>
#include <data_extractor.hpp>

namespace r2d2::thermal_camera {
    class pixel_manipulator_c {
    protected:
        std::array<std::array<float, 32>, 24> &pixels;
        mlx_parameters_s &params;
        // Pattern which will help us to automatically switch between both
        // subpages
        int patron;

    public:
        pixel_manipulator_c(mlx_parameters_s &params,
                            std::array<std::array<float, 32>, 24> &pixels);

        virtual void calculate_pixel(int row, int col) = 0;
        /**
         * Sets the patron parameter, to switch between subpages.
         * 
         * @param int row 
         * @param int col
         * @param reading_pattern pattern the current reading pattern
         */
        void set_patron(int row, int col, const reading_pattern pattern);
    };
} // namespace r2d2::thermal_camera
