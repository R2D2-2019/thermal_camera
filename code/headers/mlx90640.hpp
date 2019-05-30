#pragma once

#include <i2c_bus.hpp>

namespace r2d2::thermal_camera {
    // MLX90640 16 bits registers
    enum : uint16_t { CONTROL_REGISTER = 0x800D, STATUS_REGISTER = 0x8000 };

    enum class reading_pattern { INTERLEAVED_MODE, CHESS_PATTERN_MODE };

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
        uint16_t read_register(const uint16_t internal_address) const;

        /**
         * Writes a register.
         *
         * @param uint16_t internal_address of the chip.
         * @param uint16_t data to be written*/
        void write_register(const uint16_t internal_address,
                            const uint16_t data) const;

        /**
         * Gets a subpage from the chip and stores it into pixels[][]
         * This is pure raw data and needs to be processed.
         * */
        void set_raw_pixels();

        /**
         * Changes nth bit to to in source.
         *
         * @param int source. The to be changed value.
         * @param int n. The nth bit to be toggled.
         * @param int to. This value represents the value it has to be changed
         * to. Either has to be a 0 or a 1.
         * */
        void toggle_nth_bit(uint16_t &source, const uint8_t n,
                            const uint8_t to) const;
        // Default address of the MLX90460
        static constexpr uint8_t I2C_ADDRESS = 0x33;

    public:
        /**
         * MLX90640 constructor.
         *
         * @param i2c_bus_c
         * @param uint16_t refresh_rate (in Hz) default set to 2. Valid values
         * are (Hz): 64, 32, 16, 8, 4, 2, 1.
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
        void set_refresh_rate(uint16_t refresh_rate) const;

        /**
         * Gets the current refresh rate.
         *
         * @return uint16_t refresh rate in Hz.
         * */
        uint16_t get_refresh_rate() const;
        /**
         * Checks wether a new data set (subpage/frame) is available, and sets
         * the appropriate internal bit to 0 again.
         *
         * @return true if new data is available, false otherwise.
         * */
        bool frame_available() const;
        // Max refresh rate.

        /**
         * Sets the reading pattern of the chip. Interleaved (TV) mode or Chess
         * pattern mode. When chess pattern mode is selected, pixels in the row
         * 0, 2, 4, 6, 8, 10 etc are read first, then 1, . In Interleaved mode,
         * pixels in the column 0, 2, 4, 6, 8, 10 etc are read first. Then 1, 3,
         * 5 etc. Chess pattern is default on the chip and is preffered since
         * this results in better fixed pattern noise behaviour of the sensor.
         *
         * @param reading_pattern
         * */
        void set_reading_pattern(const reading_pattern &pattern);

        // Max refresh rate of the chip
        static constexpr uint16_t MAX_REFRESH_RATE = 64;
    };

} // namespace r2d2::thermal_camera