#pragma once

#include <data_extractor.hpp>
#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>

namespace r2d2::thermal_camera {
    /**
     * A static_var_c class is used to extract parameters from EEPROM after the
     * device has started up. These extracted parameters are then stored in
     * mlx_parameters_s. This only have to be done after startup.
     */
    class static_var_c {
    protected:
        mlx90640_i2c_c &bus;
        mlx_parameters_s &params;

    public:
        /**
         * @param mlx90640_i2c_bus bus - i2c bus for mlx90640.
         * @param mlx_parameters_s params - the mlx90640 EEPROM parameters.
         */
        static_var_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        /**
         * Extracts EEPROM data from the MLX90640 chip and stores it into
         * MLX90640Parameters
         */
        virtual void extract() = 0;
    };
} // namespace r2d2::thermal_camera
