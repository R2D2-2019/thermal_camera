#include <mlx90640_i2c.hpp>

namespace r2d2::thermal_camera {
    mlx90640_i2c_c::mlx90640_i2c_c(i2c::i2c_bus_c &bus, const uint8_t address)
        : bus(bus), address(address) {
    }

   uint16_t mlx90640_i2c_c::read_register(const uint16_t internal_address) 
                                          const {
        uint8_t raw_data[2];
        bus.read(address, raw_data, 2, internal_address, 2);
        uint16_t msb = static_cast<uint16_t>(raw_data[1]),
                 lsb = static_cast<uint16_t>(raw_data[0]);
        return msb << 8 | lsb;
    }

    void mlx90640_i2c_c::write_register(const uint16_t internal_address,
                                        const uint16_t data) const {
        uint8_t array_data[2] = {static_cast<uint8_t>(data & 0xFF),
                                 static_cast<uint8_t>((data >> 8) & 0xFF)};
        bus.write(address, array_data, 2, internal_address, 2);
    }
} // namespace r2d2::thermal_camera
