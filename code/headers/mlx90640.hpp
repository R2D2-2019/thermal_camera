#pragma once

#include <i2c_bus.hpp>
#include <mlx90640_i2c.hpp>
#include <mlx90640_processor.hpp>

namespace r2d2::thermal_camera {

    class mlx90640_c {
    private:
        /**
         * i2c bus with read and write operations specially implemented for this
         * chip.
         */
        mlx90640_i2c_c mlx_i2c_bus;
        /**
         * This object is responsible for all calulations. Best to keep it
         * seperate.
         */
        mlx90640_processor_c mlx_processor;

        /**
         * Clears the 'new data available' bit in the internal status register.
         */
        void reset_frame() const;

        /**
         * Changes nth bit to 'to' in 'source'.
         *
         * @param uint16_t source. The to be changed value.
         * @param uint8_t n. The nth bit to be toggled.
         * @param bool to. This value represents the value it has to be changed
         * to. Either has to be a 0 or a 1.
         */
        void toggle_nth_bit(uint16_t &source, const uint8_t n,
                            const bool to) const;
        /**
         * Checks wether a new data set (subpage/frame) is available, and sets
         * the appropriate internal bit to 0 again.
         *
         * @return true if new data is available, false otherwise.
         */
        bool frame_available() const;

    public:
        /**
         * MLX90640 constructor.
         *
         * @param i2c_bus_c
         * @param uint8_t address of the device. Default set with 0x33.
         * @param int refresh_rate in Hz
         * @param float emissivity correction. Default set by 1.
         */
        mlx90640_c(i2c::i2c_bus_c &bus, float emissivity = 1,
                   int refresh_rate = 2, const uint8_t address = I2C_ADDRESS);

        /**
         * Sets the refresh rate in Hz of the camera.
         *
         * @param uint16_t refresh_rate. Valid values are (Hz): 64, 32, 16, 8,
         * 4, 2, 1
         */
        void set_refresh_rate(uint16_t refresh_rate) const;

        /**
         * Gets the current refresh rate.
         *
         * @return uint16_t refresh rate in Hz.
         */
        uint16_t get_refresh_rate() const;

        /**
         * Sets the reading pattern of the chip. Interleaved (TV) mode or Chess
         * pattern mode. When chess pattern mode is selected, pixels in the row
         * 0, 2, 4, 6, 8, 10 etc are read first, then 1, 3, 5, 7. In Interleaved
         * mode, pixels in the column 0, 2, 4, 6, 8, 10 etc are read first. Then
         * 1, 3, 5 etc. Chess pattern is default on the chip and is preferred
         * since this results in better fixed pattern noise behaviour of the
         * sensor.
         *
         * @param reading_pattern
         */
        void set_reading_pattern(const reading_pattern &pattern);

        /**
         * Gets the reading pattern of the chip.
         *
         * @return reading_pattern - the read pattern from the chip.
         */
        reading_pattern get_reading_pattern() const;

        /**
         * Sets the mlx90640::pixels frame.
         */
        void set_frame();

        /**
         * Gets the temperature frame.
         *
         * @return std::array<std::array<float, 32>, 24>* the frame pointer.
         */
        std::array<std::array<float, 32>, 24> *get_frame_ptr();

        // Max refresh rate of the chip
        static constexpr uint16_t MAX_REFRESH_RATE = 64;
        // Default address of the MLX90460
        static constexpr uint8_t I2C_ADDRESS = 0x33;
    };

} // namespace r2d2::thermal_camera