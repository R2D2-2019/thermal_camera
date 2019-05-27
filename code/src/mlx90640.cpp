#include "mlx90640.hpp"

namespace r2d2::thermal_camera {

    mlx90640_c::mlx90640_c(i2c::i2c_bus_c &bus, const uint8_t address)
        : bus(bus), address(address) {
        hwlib::wait_ms(1000);
    }

    uint16_t mlx90640_c::read_register(uint16_t internal_address) {
        uint8_t raw_data[2];
        bus.read(address, raw_data, 2, internal_address,
                 2); // first 2 is 2 because we want to read 2 bytes.
        return (raw_data[1] << 8) | raw_data[0];
    }

    uint16_t mlx90640_c::getreg(uint16_t internal_address) {
        return read_register(internal_address);
    }

} // namespace r2d2::thermal_camera