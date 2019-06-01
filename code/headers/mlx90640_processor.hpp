#pragma once

#include <array>
#include <mlx90640_i2c.hpp>

namespace r2d2::thermal_camera {
    /** MLX90640 16 bits registers
     * INTERNAL means internal register.
     * EE means EEPROM (Electric Erasable Programmable Read-Only)
     * RAM means just RAM (Random Acces Memory)
     **/
    enum : uint16_t {
        INTERNAL_CONTROL_REGISTER = 0x800D,
        INTERNAL_STATUS_REGISTER = 0x8000,
        EE_VDD_PIX = 0x2433,
        RAM_VDD_PIX = 0x072A,
        EE_GAIN = 0x70A,
        RAM_GAIN = 0x2430
    };

    // This class is responsible for all calculations & includes all math.
    class mlx90640_processor_c {
    private:
        // i2c bus with (internal) read- and write_register operations.
        mlx90640_i2c_c &bus;

        // Pixel array
        std::array<std::array<uint16_t, 32>, 24> pixels;

        /**
         * By default this is 33. But should be 3.3V. This is times 10 to
         * prevent floating point values. A division by 10 will be necessary
         * when calculating with this value.
         * */
        static constexpr uint8_t VDD0 = 33;
        /**
         * Reads a block of memory (pixel values) from the chip and inserts it
         * into pixels[][].
         * */
        void set_and_read_raw_pixels();

    public:
        mlx90640_processor_c(mlx90640_i2c_c &bus);

        /**
         * Gets the VDD sensor parameters
         *
         * @return int16_t
         * */
        int16_t get_delta_V();

        /**
         * Gets the GAIN coefficient
         *
         * @return in16_t
         * */
        int16_t get_gain();
    };
} // namespace r2d2::thermal_camera