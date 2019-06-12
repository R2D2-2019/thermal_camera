#include <data_extractor.hpp>

namespace r2d2::thermal_camera {
    int data_extractor_s::apply_treshold(const int value,
                                         const uint16_t exceeds,
                                         const int minus) {
        if (value > exceeds) {
            return value - minus;
        }
        return value;
    }

    int data_extractor_s::extract_and_treshold(const int data,
                                               const uint16_t and_bits,
                                               const uint8_t shifted,
                                               const uint16_t exceeds,
                                               const int minus) {
        int extracted_data = extract_data(data, and_bits, shifted);
        extracted_data = apply_treshold(extracted_data, exceeds, minus);
        return extracted_data;
    }

    int data_extractor_s::extract_data(const int value, const uint16_t and_bits,
                                       const uint8_t shifted) {
        return (value & and_bits) >> shifted;
    }

    void data_extractor_s::check_within_limits(int &row, int &col) {
        if (row > 24) {
            row = 24;
        } else if (row == 0) {
            row = 1;
        }
        if (col > 32) {
            col = 32;
        } else if (col == 0) {
            col = 1;
        }
    }
} // namespace r2d2::thermal_camera