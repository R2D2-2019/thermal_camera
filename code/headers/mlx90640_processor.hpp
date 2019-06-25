#pragma once

#include <array>
#include <dynamic_vars/dynamic_vars.hpp>
#include <lookupables/lookupables.hpp>
#include <mlx90640_processor.hpp>
#include <pixel_manipulators/pixel_manipulators.hpp>
#include <registers.hpp>
#include <static_vars/static_vars.hpp>

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

        // Pixel calculators, calculating pixel values
        std::array<pixel_manipulator_c *, 3> pixel_calculators;

        // Pixels
        std::array<std::array<float, 32>, 24> pixels;

        // Datasheet section 11.1.3
        pix_os_ref_c pix_offset;
        // Datasheet section 11.1.4
        alpha_c alpha;
        // Datasheet section 11.1.5
        kv_c kv;
        // Datasheet section 11.1.6
        kta_c kta;
        // Lookuptables, static as well
        std::array<lookupable_c *, 4> lookupables;
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

        static constexpr int MLX_MAX_CLOCK_SPEED = 1'000'000;

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

        /**
         * Gets the pointer to the array.
         *
         * @return std::array<std::arrray, 32>, 24> the pixel values in celsius
         * degrees.
         */
        std::array<std::array<float, 32>, 24> *get_frame_ptr();
    };
} // namespace r2d2::thermal_camera