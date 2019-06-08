#pragma once

#include <array>
#include <cmath>
#include <mlx90640_i2c.hpp>
#include <registers.hpp>

namespace r2d2::thermal_camera {
    /**
     * This class is responsible for all calculations & includes all math.
     * Floating point values are used because if we cast it to integer values, a
     * rounding will take place that, at the end of all calculations, will be
     * very different from the actual desired result. Please see the datasheet.
     */
    class mlx90640_processor_c {
    private:
        // i2c bus with (internal) read- and write_register operations.
        mlx90640_i2c_c &bus;

        /**
         * Supply voltage in Volts
         */
        static constexpr float VDD0 = 3.3;
        // Kvdd, required for calculations
        int Kvdd;
        // Kvdd25, required for calculations
        int Vdd25;

        /**
         * The device is calibrated with default resolution setting = 2
         * i.e. if the one choose to change the ADC resolution setting to a
         * different one a correction of the data must be done.
         */
        uint8_t get_resolution_correlation() const;

        /**
         * Extracts the data from a given addres.
         *
         * @param uin16_t reg_addr - a register address of the chip.
         * @param uint16_t and_bits - ands the result from the read operation
         * with and_bits.
         * @param uint16_t shifted - shifts the bits with shifted amount.
         */
        uint16_t extract_data(const registers reg_addr, const uint16_t and_bits,
                              const uint8_t shifted) const;

        /**
         * Checks wether value exceeds 'exceeds', if so, it reduces value by
         * 'minus' amount.
         *
         * @param int value - the value to be checked.
         * @param uint16_t exceeds - the value it may not exceed.
         * @param uint16_t minus - the value 'value' gets reduced by.
         */
        void apply_treshold(int &value, const uint16_t exceeds,
                            const int minus) const;

        /**
         * Reads a register, extracts the data and applies a treshold on it.
         * See apply_treshold and extract_data functions for the parameter info.
         *
         * @return int - the processed data.
         */
        int get_compensated_data(const registers reg_addr,
                                 const uint16_t and_bits, const uint8_t shifted,
                                 const uint16_t exceeds, const int minus) const;

    public:
        mlx90640_processor_c(mlx90640_i2c_c &bus);

        /**
         * Gets the VDD sensor parameters
         *
         * @return float
         */
        float get_Vdd();

        /**
         * Gets the ambient temperature of the pixels
         *
         * @return float
         */
        float get_Ta() const;

        /**
         * Gets the gain parameter.Please note that this value is updated every
         * frame and it is the same for all pixels including CP regardless the
         * subpage number
         *
         * @return float
         */
        float get_gain() const;
    };
} // namespace r2d2::thermal_camera