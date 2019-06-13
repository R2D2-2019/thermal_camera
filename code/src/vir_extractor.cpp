#include <vir_extractor.hpp>

namespace r2d2::thermal_camera {
    vir_extractor_c::vir_extractor_c(mlx90640_i2c_c &bus)
        : mlx_extractor_c(bus) {
    }

    void vir_extractor_c::extract(mlx_parameters_s &params) {
        const float Vir_emissivity_comp = params.Pix_Os / params.emissivity;
        params.Vir_row_col_comp =
            Vir_emissivity_comp -
            params.TGC * ((1 - params.patron) * params.pix_os_cp_sp0 +
                          params.patron * params.pix_os_cp_sp1);
    }
} // namespace r2d2::thermal_camera