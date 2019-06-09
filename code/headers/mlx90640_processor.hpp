#pragma once

#include <mlx90640_i2c.hpp>
#include <registers.hpp>

namespace r2d2::thermal_camera {

    enum class reading_pattern { INTERLEAVED_MODE, CHESS_PATTERN_MODE };

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

        // Current reading pattern.
        reading_pattern pattern;

        /*
         * Supply voltage in Volts
         */
        static constexpr float VDD0 = 3.3;

        /**
         * TA0, datasheets refers to 25.
         */
        static constexpr uint8_t TA0 = 25;

        // Kvdd, required for calculations
        int Kvdd;

        // Kvdd25, required for calculations
        int Vdd25;

        // KstaEE common for all pixels
        int KstaEE;

        // Vdd required for calculations
        float Vdd;

        // Kgain for gain compenstation
        float Kgain;

        // Kta, required for calculations
        float Kta_row_col;

        // Ta, ambient temp, required for calculations
        float Ta;

        // Pixel gain corner pixel subpage 0
        float pix_gain_cp_sp0;

        // Pixel gain corner pixel subpage 1
        float pix_gain_cp_sp1;

        // Pixel offset corner subpage 0
        float pix_os_cp_sp0;

        // Pixel offset corner subpage 1
        float pix_os_cp_sp1;

        // TGC for calcs
        float TGC;

        /**
         * Emissivity compensation: We assume Emissivity = 1.
         * Emissivity coefficient is user defined and it is not stored in the
         * device EEPROM)
         */
        float emissivity;

        /**
         * The device is calibrated with default resolution setting = 2
         * i.e. if the one choose to change the ADC resolution setting to a
         * different one a correction of the data must be done.
         */
        uint8_t get_resolution_correlation() const;

        /**
         * Gets the VDD sensor parameters. Common for all pixels.
         *
         * @return float
         */
        void set_Vdd();

        /**
         * Gets the ambient temperature of the pixels. Common for all pixels.
         *
         * @return float
         */
        void set_Ta();

        /**
         * Sets the gain parameter. Please note that this value is updated every
         * frame and it is the same for all pixels including CP regardless the
         * subpage number
         */
        void set_Kgain();
        /*
         * Gets the Kv coeffecient
         *
         * @return float - Kv
         */
        float get_Kv_coefficient(int row, int col, uint16_t offset_addr);

        /**
         * The first step of the data processing on raw IR data is always the
         * gain compensation, regardless of pixel or subpage number.
         *
         * @param int row - the selected row. Value between 1 and 24
         * @param int col - the selected column. Value between 1 and 32
         * @return float - the compensated processed gain compensation
         */
        float get_pix_gain(const int row, const int col) const;

        /**
         * Gets the compensation of the gain of the cp (corner pixel)
         *
         * @return float - the gain for the corner pixels
         */
        void set_cp_gain();

        /**
         * As stated in "Reading patterns" the device can work in two different
         * readings modes (Chess pattern – the default one and IL (Interleave
         * mode)). Depending on the device measurement mode and pixelNumber = 1
         * ... 768 we can define a pattern which will help us to automatically
         * switch between both subpages.
         */
        int get_patron(int row, int col) const;

        /**
         * Compensates the offset, Ta and VDD of the CP.
         */
        void compensate_cp();

        /**
         * Sets the TGC parameter
         */
        void set_TGC();

        /**
         * Gets the pixel OS.
         *
         * @return float
         */
        float get_pix_OS(int row, int col);

        /**
         * Sets the Ksta parameter
         */
        void set_Ksta_EE();

        /**
         * Gets the pixel number.
         * @return int
         */
        int get_pixel_number(int row, int col) const;

        /**
         * Gets the alpha compensation
         *
         * @param int row - the pixel row
         * @param int col - the pixel column
         * @param float Pix_Os
         * @param int patron - the pattern
         * @return float
         */
        float get_alpha_compensation(const int row, const int col,
                                     const float Pix_OS,
                                     const int patron) const;

        /**
         * Gets the Vir row col compensation
         *
         * @param float Pix_Os
         * @param int patron - the pattern
         * @return float
         */
        float get_IR_gradient_compensation(const float Pix_Os,
                                           const int patron) const;

        /**
         * Returns the pixel temperature in Celsius degrees
         *
         * @param float Vir_row_col_comp ir compensation
         * @param float alpha_comp alpha compensation
         * @return float temperature in celsius
         */
        float get_pixel_temp(float Vir_row_col_comp, float alpha_comp) const;

    public:
        mlx90640_processor_c(mlx90640_i2c_c &bus, float emissivity = 1);

        /**
         * Returns the resoting offset.
         *
         * @return int - the offset
         */
        float get_offset_calculation(const int row, const int col);

        /**
         * Sets the reading pattern. Required, because when a different pattern
         * is used, a different calculation is required.
         */
        void set_reading_pattern(const reading_pattern &pattern);
    };
} // namespace r2d2::thermal_camera