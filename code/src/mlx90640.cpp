#include <cmath>
#include <mlx90640.hpp>

namespace r2d2::thermal_camera {

    mlx90640_c::mlx90640_c(i2c::i2c_bus_c &bus, const uint16_t refresh_rate,
                           const uint8_t address)
        : bus(bus), address(address) {
        hwlib::wait_ms(1000);
        set_refresh_rate(refresh_rate);
    }

    uint16_t mlx90640_c::read_register(const uint16_t internal_address) {
        uint8_t raw_data[2];
        bus.read(address, raw_data, 2, internal_address, 2);
        return static_cast<uint16_t>((raw_data[1] << 8) | raw_data[0]);
    }

    void mlx90640_c::write_register(const uint16_t internal_address,
                                    const uint16_t data) {
        uint8_t array_data[] = {static_cast<uint8_t>(data),
                                static_cast<uint8_t>(data >> 8)};
        bus.write(I2C_ADDRESS, array_data, 2, internal_address, 2);
    }

    void mlx90640_c::get_raw_pixels() {
    }

    void mlx90640_c::toggle_nth_bit(uint16_t &source, const uint8_t n,
                                    const uint8_t to) {
        source = (source & ~(1U << n)) | (to << n);
    }

    void mlx90640_c::set_refresh_rate(uint16_t refresh_rate) {
        if (refresh_rate > MAX_REFRESH_RATE) {
            return;
        }
        refresh_rate =
            static_cast<uint16_t>(1 + std::log2(refresh_rate));

        uint16_t data = read_register(CONTROL_REGISTER);

        for (uint8_t i = 0; i < 3; i++) {
            toggle_nth_bit(data, i + 7, refresh_rate >> i & 1);
        }
        write_register(CONTROL_REGISTER, data);
    }

    uint16_t mlx90640_c::get_refresh_rate() {
        uint16_t rate = read_register(CONTROL_REGISTER);
        rate >>= 7 & 7;
        return 1 << (rate - 1);
    }

} // namespace r2d2::thermal_camera
