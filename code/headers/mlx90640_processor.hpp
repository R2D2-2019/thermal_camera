#pragma once

#include <array>
#include <mlx90640_i2c.hpp>
#include <mlx_extractor.hpp>
#include <mlx_parameters.hpp>
#include <registers.hpp>

namespace r2d2::thermal_camera {
    /**
     * This class contains the pipeline for all mlx_extractor_c calls.
     * See: "get_temperature_pixel"
     */
    class mlx90640_processor_c {
    private:
        // i2c bus with (internal) read- and write_register operations.
        mlx90640_i2c_c &bus;

        // Current reading pattern.
        reading_pattern pattern;

        // POD with all parameters
        mlx_parameters_s params;

        // Container, containing all the extractors
        std::array<mlx_extractor_c *, 10> extractors;

        /**
         * The device is calibrated with default resolution setting = 2
         * i.e. if the one choose to change the ADC resolution setting to a
         * different one a correction of the data must be done.
         */
        void set_resolution_correlation();

        /**
         * Gets the VDD sensor parameters. Common for all pixels.
         *
         * @return float
         */
        void set_Vdd();

    public:
        mlx90640_processor_c(mlx90640_i2c_c &bus, float emissivity = 1);

        /**
         * Pipeline for getting the pixel temperature it is pointed at.
         *
         * @return float the temperature at (row, col)
         */
        float get_temperature_pixel(const int row, const int col);

        /**
         * Sets the reading pattern.
         *
         * @param reading_pattern
         */
        void set_reading_pattern(const reading_pattern &pattern);
    };
} // namespace r2d2::thermal_camera