#include <dynamic_vars/vdd_var.hpp>

namespace r2d2::thermal_camera {
    vdd_var_c::vdd_var_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : dynamic_var_c(bus, params) {
    }

    void vdd_var_c::re_calculate() {
        int data;

        data = bus.read_register(registers::RAM_VDD_PIX);
        const int ram_vdd_pix = data_extractor::apply_treshold(data);
        params.Vdd =
            ((params.res_cor * ram_vdd_pix - params.Vdd25) / params.Kvdd) +
            params.VDD0;         
    }
} // namespace r2d2::thermal_camera
