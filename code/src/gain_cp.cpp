#include <gain_cp.hpp>

namespace r2d2::thermal_camera {
    gain_cp_c::gain_cp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : dynamic_var_c(bus, params) {
    }

    void gain_cp_c::re_calculate() {
        int data;
        
        data = bus.read_register(registers::RAM_CP_SP0);
        params.pix_gain_cp_sp0 = data_extractor::apply_treshold(data) * params.Kgain;

        data = bus.read_register(registers::RAM_CP_SP1);
        params.pix_gain_cp_sp1 = data_extractor::apply_treshold(data) * params.Kgain;
    }
} // namespace r2d2::thermal_camera