#pragma once

#include <array>
#include <dynamic_vars/dynamic_var.hpp>
#include <lookupables/lookupable.hpp>
#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>
#include <pixel_manipulators/pixel_manipulator.hpp>
#include <registers.hpp>
#include <static_vars/static_var.hpp>

namespace r2d2::thermal_camera {
    /**
     * This class is the pipeline.
     */
    class mlx90640_processor_c {
    private:
        // i2c bus with (internal) read- and write_register operations.
        mlx90640_i2c_c &bus;

        // Current reading pattern.
        reading_pattern pattern;

        // POD containing all parameters
        mlx_parameters_s params;

        // Container, calculating all the dynamic variables
        std::array<dynamic_var_c *, 4> dynamic_vars;

        // Static variables stored in device EEPROM
        std::array<static_var_c *, 13> static_vars;

        // Lookuptables, static as well
        std::array<lookupable_c *, 4> lookupables;

        // Pixel calculators, calculating pixel values
        std::array<pixel_manipulator_c *, 3> pixel_calculators;

        // Pixels
        std::array<std::array<float, 32>, 24> pixels;

        /**
         * Initializes one lookuptable with a double for loop.
         *
         * @param lookuptable table - a lookuptable.
         */
        void init_table(lookupable_c &table);

        /**
         * Calculates all the values for pixels.
         *
         * @param pixel_manipulator_c manipulator - a pixel manipulator.
         */
        void calculate_pixel_value(pixel_manipulator_c &manipulator);

        static constexpr int KTA = 3;
        static constexpr int KV = 2;
        static constexpr int ALPHA = 1;
        static constexpr int PIX_OS_REF = 0;

    public:
        /**
         * Constructor.
         *
         * @param mlx90640_i2c_c bus - the i2c bus for mlx90640.
         * @param float emissivity - The emissivity of the surface of a material
         * is its effectiveness in emitting energy as thermal radiation. Thermal
         * radiation is electromagnetic radiation and it may include both
         * visible radiation (light) and infrared radiation, which is not
         * visible to human eyes.
         */
        mlx90640_processor_c(mlx90640_i2c_c &bus, float emissivity = 1);

        /**
         * Pipeline for getting the pixel temperature it is pointed at.
         *
         * @return float - the temperature at (row, col)
         */
        void set_frame();

        /**
         * Sets the reading pattern.
         *
         * @param reading_pattern
         */
        void set_reading_pattern(const reading_pattern &pattern);
        

        std::array<std::array<float, 32>, 24> *get_frame_ptr();
    };
} // namespace r2d2::thermal_camera