#include <ee_off_cp.hpp>

namespace r2d2::thermal_camera {
    ee_off_cp_c::ee_off_cp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_off_cp_c::extract() {
        int data;

        data = bus.read_register(registers::EE_CP_OFF_DELTA_OFFSET_CP_SP0);
        params.off_cp_sp0 =
            data_extractor::extract_and_treshold(data, 0x03FF, 0, 511, 1024);
            
        const int off_cp_subpage_1_delta =
            data_extractor::extract_and_treshold(data, 0xFC00, 10, 31, 64);

        params.off_cp_sp1 = params.off_cp_sp0 + off_cp_subpage_1_delta;
    }
} // namespace r2d2::thermal_camera
