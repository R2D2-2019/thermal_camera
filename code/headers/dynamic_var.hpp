#pragma once

#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>
#include <data_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * A dynamic_var_c class is used to read data continuously from
     * RAM cells. RAM cells contain the varying data, thus a re-calculation will
     * be needed.
     */
    class dynamic_var_c {
    protected:
        // I2C bus fine tuned for mlx90640.
        mlx90640_i2c_c &bus;
        // Parameter struct containing calibration data.
        mlx_parameters_s &params;

    public:
        /**
         * Constructor.
         * 
         * @param mlx90640_i2c_c bus.
         * @param mlx_parameters_s parameters.
         */
        dynamic_var_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        /**
         * Recalculates the value and stores it in mlx_parameters_s.
         */
        virtual void re_calculate() = 0;
    };
} // namespace r2d2::thermal_camera
