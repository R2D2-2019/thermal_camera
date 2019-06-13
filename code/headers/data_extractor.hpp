#pragma once
#include <cstdint>

namespace r2d2::thermal_camera {
    class data_extractor_s {
    public:
        // static class
        data_extractor_s() = delete;

        /**
         * Checks wether value exceeds 'exceeds', if so, it reduces value by
         * 'minus' amount.
         *
         * @param int value - the value to be checked.
         * @param uint16_t exceeds - the value it may not exceed.
         * @param uint16_t minus - the value 'value' gets reduced by.
         *
         * @return int - the tresholded value
         */
        static int apply_treshold(const int value,
                                  const uint16_t exceeds = 32767,
                                  const int minus = 65536);

        /**
         * Extracts the data from value and applies a treshold on it.
         * See apply_treshold and extract_data functions for the parameter info.
         *
         * @return int - the extracted and tresholded data.
         */
        static int extract_and_treshold(const int data, const uint16_t and_bits,
                                        const uint8_t shifted,
                                        const uint16_t exceeds = 32767,
                                        const int minus = 65536);
        /**
         * Extracts the data from value.
         *
         * @param uin16_t value - the value to be extracted.
         * @param uint16_t and_bits - ands the result from the read operation
         * with and_bits.
         * @param uint16_t shifted - shifts the bits with shifted amount.
         *
         * @return int - the extracted data.
         */
        static int extract_data(const int value, const uint16_t and_bits,
                                const uint8_t shifted);

        /**
         * Checks wether row is higher than 32 and col higher than 24.
         * If so. row = 32 and/or col = 24.
         */
        static void check_within_limits(int &row, int &col);
    };
} // namespace r2d2::thermal_camera