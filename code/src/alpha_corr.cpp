#include <alpha_corr.hpp>

namespace r2d2::thermal_camera {
    alpha_corr_c::alpha_corr_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void alpha_corr_c::extract() {
        params.alpha_corr_range1 =
            1 / (1 + params.ksto1 * (params.ct2 - params.ct1));
        params.alpha_corr_range2 = 1;
        params.alpha_corr_range3 = 1 + params.ksto2 * (params.ct3 - params.ct2);
        params.alpha_corr_range4 =
            (1 + params.ksto2 * (params.ct3 - params.ct2)) *
            (1 + params.ksto3 * (params.ct4 - params.ct3));
    }
} // namespace r2d2::thermal_camera
