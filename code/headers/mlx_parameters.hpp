#pragma once
#include <cstdint>

namespace r2d2::thermal_camera {
    struct mlx_parameters_s {
        /*
         * Supply voltage in Volts
         */
        static constexpr float VDD0 = 3.3;
        /**
         * TA0, datasheets refers to 25.
         */
        static constexpr uint8_t TA0 = 25;
        /**
         * Emissivity compensation: We assume Emissivity = 1.
         * Emissivity coefficient is user defined and it is not stored in the
         * device EEPROM)
         */
        float emissivity;
        // Kvdd, required for calculations
        int Kvdd;
        // Kvdd25, required for calculations
        int Vdd25;
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
        // resulting pixel temperature
        float To_row_col;
        // alpha compensation (i, j)
        float alpha_comp_row_col;
        // resolution correlation
        uint8_t res_cor;
        // Kv coefficient
        float Kv;
        // pixel offset
        float Pix_Os;
        // patron reading (interleaved or chess) result
        int patron;
        // Vir (i, j) compensated
        float Vir_row_col_comp;
    };
} // namespace r2d2::thermal_camera
