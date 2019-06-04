#pragma once

#include <array>
#include <cmath>
#include <mlx90640_i2c.hpp>

/**
 * MLX90640 16 bits registers
 * INTERNAL means internal register.
 * EE means EEPROM (Electric Erasable Programmable Read-Only)
 * RAM means just RAM (Random Acces Memory)
 **/
enum : uint16_t {
    INTERNAL_CONTROL_REGISTER = 0x800D,
    INTERNAL_STATUS_REGISTER = 0x8000,

    RAM_TA_VBE = 0x0700,
    RAM_GAIN = 0x070A,
    RAM_VDD_PIX = 0x072A,
    RAM_TA_PTAT = 0x0720,

    EE_GAIN = 0x2430,
    EE_PTAT25 = 0x2431,
    EE_VDD_PIX = 0x2433,
    EE_SCALE_OCC = 0x2410,
    EE_RESOLUTION = 0x2438,
    EE_KV_KT_PTAT = 0x2432
};

namespace r2d2::thermal_camera {
    /**
     * This class is responsible for all calculations & includes all math.
     * Floating point values are used because if we cast it to integer values, a
     * rounding will take place that, at the end of all calculations, will be
     * very different from the actual desired result. Please see the datasheet.
     * */
    class mlx90640_processor_c {
    private:
        // i2c bus with (internal) read- and write_register operations.
        mlx90640_i2c_c &bus;

        // Pixel array
        std::array<std::array<uint16_t, 32>, 24> pixels;

        /**
         * Supply voltage in Volts
         * */
        static constexpr float VDD0 = 3.3;
        // Kvdd, required for calculations
        int Kvdd;
        // Kvdd25, required for calculations
        int Vdd25;

        /**
         * Reads a block of memory (pixel values) from the chip and inserts it
         * into pixels.
         * */
        void set_and_read_raw_pixels();

        /**
         * The device is calibrated with default resolution setting = 2
         * i.e. if the one choose to change the ADC resolution setting to a
         * different one a correction of the data must be done.
         * */
        uint8_t get_resolution_correlation() const;

        /**
         * Extracts the data from a given addres.
         *
         * @param uin16_t reg_addr - a register address of the chip.
         * @param uint16_t and_bits - ands the result from the read operation
         * with and_bits.
         * @param uint16_t shifted - shifts the bits with shifted amount.
         * */
        uint16_t extract_data(const uint16_t reg_addr, const uint16_t and_bits,
                              const uint8_t shifted) const;

        /**
         * Checks wether value exceeds 'exceeds', if so, it reduces value by
         * 'minus' amount.
         *
         * @param int value - the value to be checked.
         * @param uint16_t exceeds - the value it may not exceed.
         * @param uint16_t minus - the value 'value' gets reduced by.
         * */
        void apply_treshold(int &value, const uint16_t exceeds,
                            const int minus) const;

        /**
         * Reads a register, extracts the data and applies a treshold on it.
         * See apply_treshold and extract_data functions for the parameter info.
         *
         * @return int - the processed data.
         * */
        int get_compensated_data(const uint16_t reg_addr,
                                 const uint16_t and_bits, const uint8_t shifted,
                                 const uint16_t exceeds, const int minus) const;

    public:
        mlx90640_processor_c(mlx90640_i2c_c &bus);

        /**
         * Gets the VDD sensor parameters
         *
         * @return float
         * */
        float get_Vdd();

        /**
         * Gets the ambient temperature of the pixels
         *
         * @return float
         * */
        float get_Ta() const;

        /**
         * Gets the gain parameter.Please note that this value is updated every
         * frame and it is the same for all pixels including CP regardless the
         * subpage number
         *
         * @return float
         * */
        float get_gain() const;
    };
} // namespace r2d2::thermal_camera