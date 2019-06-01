#include <cmath>
#include <mlx90640.hpp>

namespace r2d2::thermal_camera {

    mlx90640_c::mlx90640_c(i2c::i2c_bus_c &bus, const uint8_t address)
        : mlx_i2c_bus(bus, address), mlx_processor(mlx_i2c_bus) {
    }

    void mlx90640_c::toggle_nth_bit(uint16_t &source, const uint8_t n,
                                    const uint8_t to) const {
        source = (source & ~(1U << n)) | (to << n);
    }

    uint16_t mlx90640_c::get_refresh_rate() const {
        uint16_t rate = mlx_i2c_bus.read_register(INTERNAL_CONTROL_REGISTER);
        rate >>= 7 & 7;
        return 1 << (rate - 1);
    }

    void mlx90640_c::set_refresh_rate(uint16_t refresh_rate) const {
        if (refresh_rate > MAX_REFRESH_RATE) {
            return;
        }
        refresh_rate = static_cast<uint16_t>(1 + std::log2(refresh_rate));
        uint16_t data = mlx_i2c_bus.read_register(INTERNAL_CONTROL_REGISTER);
        for (uint8_t i = 0; i < 3; i++) {
            toggle_nth_bit(data, i + 7, refresh_rate >> i & 1);
        }
        mlx_i2c_bus.write_register(INTERNAL_CONTROL_REGISTER, data);
    }

    bool mlx90640_c::frame_available() const {
        uint16_t data = mlx_i2c_bus.read_register(INTERNAL_STATUS_REGISTER);
        // Checks wether the 3rd bit is 1 or 0.
        const bool frame_available = (data >> 2) & 1;
        if (!frame_available) {
            // frame available is false, so we don't need to
            // toggle anything back. We can leave as it was.
            return false;
        }
        // Otherwise, mark bit as read by setting it back to 0
        toggle_nth_bit(data, 3, 0);
        mlx_i2c_bus.write_register(INTERNAL_STATUS_REGISTER, data);
        return frame_available;
    }

    void mlx90640_c::set_reading_pattern(const reading_pattern &pattern) const {
        uint16_t data = mlx_i2c_bus.read_register(INTERNAL_CONTROL_REGISTER);
        uint8_t bit;
        switch (pattern) {
        case reading_pattern::CHESS_PATTERN_MODE:
            bit = 1;
            break;
        case reading_pattern::INTERLEAVED_MODE:
            bit = 0;
            break;
        default:
            return;
        }
        toggle_nth_bit(data, 12, bit);
        mlx_i2c_bus.write_register(INTERNAL_CONTROL_REGISTER, data);
    }

} // namespace r2d2::thermal_camera
