#pragma once
#include <mlx90640_i2c.hpp>
#include <mlx_parameters.hpp>

namespace r2d2::thermal_camera {
    enum class reading_pattern { INTERLEAVED_MODE, CHESS_PATTERN_MODE };

    class mlx_extractor_c {
    protected:
        mlx90640_i2c_c &bus;
        /**
         * Gets the pixel number.
         *
         * @param int row
         * @param int col
         * @return int
         */
        int get_pixel_number(int row, int col) const;

    public:
        mlx_extractor_c(mlx90640_i2c_c &bus);
        virtual void extract(mlx_parameters_s &params) = 0;
    };
} // namespace r2d2::thermal_camera
