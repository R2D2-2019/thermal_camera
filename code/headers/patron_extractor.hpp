#pragma once

#include <mlx_extractor.hpp>

namespace r2d2::thermal_camera {
    /**
     * As stated in "Reading patterns" the device can work in two different
     * readings modes (Chess pattern – the default one and IL (Interleave
     * mode)). Depending on the device measurement mode and pixelNumber = 1
     * ... 768 we can define a pattern which will help us to automatically
     * switch between both subpages.
     */
    class patron_extractor_c : public mlx_extractor_c {
    private:
        int row, col;
        reading_pattern pattern;

    public:
        patron_extractor_c(mlx90640_i2c_c &bus, int row, int col,
                           const reading_pattern pattern);

        void extract(mlx_parameters_s &params) override;
    };
} // namespace r2d2::thermal_camera
