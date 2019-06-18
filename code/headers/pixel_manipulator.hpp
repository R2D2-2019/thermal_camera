#pragma once

#include <array>
#include <data_extractor.hpp>
#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>
#include <reading_patterns.hpp>

namespace r2d2::thermal_camera {
    /**
     * The pixel manipulator class is a class that does certain calculations
     * with the pixels. The implementation is up to the inherited class.
     */
    class pixel_manipulator_c {
    protected:
        // A reference to the current pixels
        std::array<std::array<float, 32>, 24> &pixels;
        mlx_parameters_s &params;
        // Pattern which will help us to automatically switch between both
        // subpages
        int patron;

    public:
        /**
         * @param mlx_parameters_s params the mlx90640 eeprom data
         * @param std::array<std::array<float, 32>, 24> pixels a reference to
         * the current calculated pixels
         */
        pixel_manipulator_c(mlx_parameters_s &params,
                            std::array<std::array<float, 32>, 24> &pixels);

        /**
         * Pure virtual method for calculating certain pixel values given a row
         * and col.
         *
         * @param unsigned int row - a pixel row
         * @param unsigned int col - a pixel col
         */
        virtual void calculate_pixel(unsigned int row, unsigned int col) = 0;

        /**
         * Sets the patron parameter, to automatically switch between subpages.
         *
         * @param unsigned int row
         * @param unsigned int col
         * @param reading_pattern pattern the current reading pattern
         */
        void set_patron(unsigned int row, unsigned int col,
                        const reading_pattern pattern);
    };
} // namespace r2d2::thermal_camera
