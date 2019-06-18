#pragma once

#include <array>
#include <data_extractor.hpp>
#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>


namespace r2d2::thermal_camera {
    /**
     * A lookupable is almost the same as a static_var_c except that it contains
     * a container of floating point values. Each pixel(i, j) has a different
     * calibration value, that is why a lookuptable will increase performance.
     */
    class lookupable_c {
    protected:
        // The lookuptable 32x24 pixels.
        std::array<std::array<float, 32>, 24> table;
        // I2C bus for mlx90640
        mlx90640_i2c_c &bus;
        // MLX parameter calibration struct.
        mlx_parameters_s &params;
        
        /**
         * Gets the pixel number.
         * 
         * @param int row
         * @param int col
         * @return int the pixel number (starting from 0x0400)
         */
        int get_pixel_number(int row, int col) const;

    public:
        lookupable_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);
        /**
         * Sets the calibration parameter in table, given a row, col.
         * 
         * @param int row the pixel row
         * @param int col the pixel col
         */
        virtual void calculate_pixel(int row, int col) = 0;
        /**
         * Gets the calibration data specifically for a given row, col.
         * 
         * @param int row the row of the calibration table.
         * @param int col the col of the calibration table.
         * @return float the calibration data for pixel(row, col)
         */
        float get_value(int row, int col);
    };
} // namespace r2d2::thermal_camera