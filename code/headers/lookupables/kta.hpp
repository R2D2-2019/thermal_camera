#pragma once

#include <lookupables/lookupable.hpp>

namespace r2d2::thermal_camera {
    class kta_c : public lookupable_c {
    private:
        int Kta_scale_1;
        int Kta_scale_2;
    public:
        kta_c(mlx90640_i2c_c &bus, mlx_parameters_s &params);
        
        void calculate_pixel(unsigned int row, unsigned int col) override;
    };
}