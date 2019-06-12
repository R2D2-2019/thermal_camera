#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * Sets the pixel OS (offset).
     */
    class pix_os_c : public mlx_extractor_c {
    private:
        int row, col;
        /**
         * The first step of the data processing on raw IR data is always the
         * gain compensation, regardless of pixel or subpage number.
         *
         * @param int row - the selected row. Value between 1 and 24
         * @param int col - the selected column. Value between 1 and 32
         * @param mlx_parameter_s params - the mlx struct
         * @return float - the compensated processed gain compensation
         */
        float get_pix_gain(const int row, const int col,
                           mlx_parameters_s &params) const;

    public:
        pix_os_c(mlx90640_i2c_c &bus, int row, int col);
        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
