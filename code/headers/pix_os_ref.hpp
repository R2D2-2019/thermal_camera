#pragma once

#include <lookupable.hpp>

namespace r2d2::thermal_camera {
    class pix_os_ref_c : public lookupable_c {
    private:
        int offset_average;
        int Occ_scale_row;
        int Occ_scale_col;
        int Occ_scale_rem;
    public:
        pix_os_ref_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);

        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
} // namespace r2d2::thermal_camera
