#pragma once

#include <mlx90640_i2c.hpp>

/**
 * MLX90640 16 bits registers
 * INTERNAL means internal register.
 * EE means EEPROM (Electric Erasable Programmable Read-Only)
 * RAM means just RAM (Random Acces Memory)
 */
enum : uint16_t {
    INTERNAL_CONTROL_REGISTER = 0x800D,
    INTERNAL_STATUS_REGISTER = 0x8000,

    RAM_PAGE_START = 0x0400,
    RAM_TA_VBE = 0x0700,
    RAM_CP_SP0 = 0x708,
    RAM_CP_SP1 = 0x0728,
    RAM_GAIN = 0x070A,
    RAM_VDD_PIX = 0x072A,
    RAM_TA_PTAT = 0x0720,

    EE_SCALE_OCC = 0x2410,
    EE_PIX_OS_AVERAGE = 0x2411,
    EE_OCC_ROWS_START = 0x2412,
    EE_OCC_COLS_START = 0x2418,
    EE_ACC_ROW_COL = 0x241F,
    EE_ALPHA_ACC_SCALE = 0x2420,
    EE_PIX_SENSITIVITY_AVG = 0x2421,
    EE_ACC_ROW = 0x2422,
    EE_ACC_COL = 0x0248,
    EE_GAIN = 0x2430,
    EE_PTAT25 = 0x2431,
    EE_KV_KT_PTAT = 0x2432,
    EE_VDD_PIX = 0x2433,
    EE_KV_AVG = 0x2434,
    EE_CHESS_CX = 0x2435,
    EE_KTA_AVG = 0x2436,
    EE_CTRL_CALIB_KV_KTA_SCALE = 0x2438,
    EE_CP_SP0_ALPHA = 0x2439,
    EE_CP_OFF_DELTA_OFFSET_CP_SP0 = 0x243A,
    EE_KV_KTA_CP = 0x243B,
    EE_KSTA_TGC = 0x243C,
    EE_OFFSET_PIX = 0x243F,

};

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
         * Extracts the data from a given addres.
        
         * @param uin16_t reg_addr - a register address of the chip.
         * @param uint16_t and_bits - ands the result from the read operation
         * with and_bits.
         * @param uint16_t shifted - shifts the bits with shifted amount.
        */
        uint16_t extract_data(const uint16_t reg_addr, const uint16_t and_bits,
                              const uint8_t shifted) const;

        /**
         * Checks wether value exceeds 'exceeds', if so, it reduces value by
         * 'minus' amount.
        
         * @param int value - the value to be checked.
         * @param uint16_t exceeds - the value it may not exceed.
         * @param uint16_t minus - the value 'value' gets reduced by.
        */

        void apply_treshold(int &value, const uint16_t exceeds,
                            const int minus) const;

        /**
         * Reads a register, extracts the data and applies a treshold on it.
         * See apply_treshold and extract_data functions for the parameter info.
        
         * @return int - the processed data.
        */
        int get_compensated_data(const uint16_t reg_addr,
                                 const uint16_t and_bits, const uint8_t shifted,
                                 const uint16_t exceeds, const int minus) const;

        /**
         * Performs a read on the i2c bus, checks wether the read value gets
         * higher than 32767. If so, it gets reduced by 65536. This is when no
         * shifting or other bitwise operands are necessary.
        
         * See apply_treshold and extract_data functions for the parameter info.
        */
        int read_and_apply_treshold(const uint16_t addr) const;

        /**
         * Checks wether row is higher than 32 and col higher than 24.
         * If so. row = 32 and/or col = 24.
         */
        void check_within_limits(int &row, int &col) const;

        /**
         * Gets the VDD sensor parameters. Common for all pixels.
        
         * @return float
        */
        void set_Vdd();

        /**
         * Gets the ambient temperature of the pixels. Common for all pixels.
        
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
        
         * @return float - Kv
        */
        float get_Kv_coefficient(int row, int col, uint16_t offset_addr);

        /**
         * The first step of the data processing on raw IR data is always the
         * gain compensation, regardless of pixel or subpage number.
        
         * @param int row - the selected row. Value between 1 and 24
         * @param int col - the selected column. Value between 1 and 32
         * @return float - the compensated processed gain compensation
        */
        float get_pix_gain(const int row, const int col) const;

        /**
         * Gets the compensation of the gain of the cp (corner pixel)
        
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
        Gets the pixel OS.
        
        @return float
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

    public:
        mlx90640_processor_c(mlx90640_i2c_c &bus, float emissivity = 1);

        /**
         * Returns the resoting offset.
        
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