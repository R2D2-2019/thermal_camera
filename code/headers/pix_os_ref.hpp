#pragma once

#include <lookupable.hpp>

namespace r2d2::thermal_camera {
    class pix_os_ref_c : public lookupable_c {
    private:
        float get_pix_gain(const int row, const int col,
                           mlx_parameters_s &params) const;

    public:
        pix_os_ref_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
} // namespace r2d2::thermal_camera
