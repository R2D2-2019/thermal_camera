#pragma once

#include <i2c_bus.hpp>

namespace r2d2::thermal_camera {
    // MLX90640 16 bits registers
    enum : uint16_t { CONTROL_REGISTER = 0x800D };

    class mlx90640_c {
    private:
        i2c::i2c_bus_c &bus;
        uint8_t address;
        uint16_t pixels[32][24];
        /**
         * Reads a register
         *
         * @param uint16_t internal address of the chip
         * @return uint16_t read data.
         * */
        uint16_t read_register(const uint16_t internal_address);

        /**
         * Writes a register.
         *
         * @param uint16_t internal_address of the chip.
         * @param uint16_t data to be written*/
        void write_register(const uint16_t internal_address,
                            const uint16_t data);

        /**
         * Gets a subpage from the chip and stores it into pixels[][]
         * This is pure raw data and needs to be processed.
         * */
        void get_raw_pixels();

        /**
         * Changes nth bit to to in source.
         *
         * @param int source. The to be changed value.
         * @param int n. The nth bit to be toggled.
         * @param int to. This value represents the value it has to be changed
         * to. Either has to be a 0 or a 1.
         * */
        void toggle_nth_bit(uint16_t &source, const uint8_t n,
                            const uint8_t to);
        // Default address of the MLX90460
        static constexpr uint8_t I2C_ADDRESS = 0x33;

    public:
        /**
         * MLX90640 constructor.
         *
         * @param i2c_bus_c
         * @param uint16_t refresh_rate default set to 2.
         * @param uint8_t address of the device. Default set with 0x33.
         * */
        mlx90640_c(i2c::i2c_bus_c &bus, const uint16_t refresh_rate = 2,
                   const uint8_t address = I2C_ADDRESS);
        /**
         * Sets the refresh rate in Hz of the camera.
         *
         *
         * @param uint16_t refresh_rate. Valid values are (Hz): 64, 32, 16, 8,
         * 4, 2, 1
         * */
        void set_refresh_rate(uint16_t refresh_rate);

        /**
         * Gets the current refresh rate.
         *
         * @return uint16_t refresh rate in Hz.
         * */
        uint16_t get_refresh_rate();
        // Max refresh rate.
        static constexpr uint16_t MAX_REFRESH_RATE = 64;
    };

} // namespace r2d2::thermal_camera