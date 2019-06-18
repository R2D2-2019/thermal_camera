#include <gain_comp.hpp>

namespace r2d2::thermal_camera {
    gain_comp_c::gain_comp_c(mlx90640_i2c_c &bus, mlx_parameters_s &params,
                             std::array<std::array<float, 32>, 24> &pixels)
        : pixel_manipulator_c(params, pixels), bus(bus) {
    }

    void gain_comp_c::calculate_pixel(int row, int col) {
        uint16_t addr_row_col =
            ((row - 1) << 5) + (col - 1) + registers::RAM_PAGE_START;
        int raw_ir_data = bus.read_register(addr_row_col);
        raw_ir_data = data_extractor_s::apply_treshold(raw_ir_data);
        pixels[row - 1][col - 1] = raw_ir_data * params.Kgain;
    }
} // namespace r2d2::thermal_camera
