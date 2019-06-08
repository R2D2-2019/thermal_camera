#pragma once
#include <i2c_bus.hpp>
#include <registers.hpp>

namespace r2d2::thermal_camera {
    /**
     * This class contains two methods by which reading and writing operations
     * are carried out.
     */
    class mlx90640_i2c_c {
    private:
        // i2c bus
        i2c::i2c_bus_c &bus;
        // i2c mlx90640 address
        uint16_t address;

    public:
        mlx90640_i2c_c(i2c::i2c_bus_c &bus, const uint8_t address);
        /**
         * Reads a register
         *
         * @param uint16_t internal address of the chip
         * @return uint16_t read data.
         */
        uint16_t read_register(const uint16_t internal_address) const;

        /**
         * Writes a register.
         *
         * @param uint16_t internal_address of the chip.
         * @param uint16_t data to be written
         */
        void write_register(const uint16_t internal_address,
                            const uint16_t data) const;
    };
} // namespace r2d2::thermal_camera