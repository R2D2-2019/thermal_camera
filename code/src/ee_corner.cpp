#include <ee_corner_temp.hpp>

namespace r2d2::thermal_camera {
    ee_corner_temp_c::ee_corner_temp_c(mlx90640_i2c_c &bus,
                                       mlx_parameters_s &params)
        : static_var_c(bus, params) {
    }

    void ee_corner_temp_c::extract() {
        int data = bus.read_register(registers::EE_OFFSET_PIX);
        int step = data_extractor_s::extract_data(data, 0x3000, 12) * 10;
        params.ct3 =
            data_extractor_s::extract_and_treshold(data, 0x00F0, 4) * step;
        params.ct3 =
            data_extractor_s::extract_and_treshold(data, 0x0F00, 8) * step +
            params.ct3;
    }
} // namespace r2d2::thermal_camera
