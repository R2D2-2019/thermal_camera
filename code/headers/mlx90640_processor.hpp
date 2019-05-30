#pragma once

#include <i2c_bus.hpp>

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
        /**
         * By default this is 33. But should be 3.3V. This is times 10 to
         * prevent floating point values. A division by 10 will be necessary
         * when calculating with this value.
         * */
        static constexpr uint8_t VDD0 = 33;

    protected:
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
         * Gets the VDD sensor parameters
         *
         * @return in16_t
         * */
        int16_t get_delta_V();

        /**
         * Gets the GAIN coefficient
         * */
        int16_t get_gain();

        mlx90640_processor_c(i2c::i2c_bus_c &bus, const uint8_t address);
    };
} // namespace r2d2::thermal_camera