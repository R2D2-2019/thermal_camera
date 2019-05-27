#pragma once

#include <i2c_bus.hpp>

namespace r2d2::thermal_camera {

    class mlx90640_c {
    private:
        i2c::i2c_bus_c &bus;
        uint8_t address;
        uint8_t raw_pixels[32][24];
        uint16_t read_register(uint16_t internal_address);

    public:
        mlx90640_c(i2c::i2c_bus_c &bus, const uint8_t address = 0x33);
        uint16_t getreg(uint16_t internal_address);
    };

} // namespace r2d2::thermal_camera