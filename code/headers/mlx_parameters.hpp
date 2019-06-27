#pragma once
#include <cstdint>

namespace r2d2::thermal_camera {
    struct mlx_parameters_s {
        /**
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
        // Kgain for gain compenstation
        float Kgain;
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
        // resolution correlation
        uint8_t res_cor;
        // Kvdd, required for calculations
        int Kvdd;
        // Kvdd25, required for calculations
        int Vdd25;
        // Vdd required for calculations
        float Vdd;
        // EE KTptat
        float KTptat;
        // EE KVptat
        float KVptat;
        // EE Vptat25
        int Vptat25;
        // EE alpha_ptat
        float alpha_ptat;
        // EE gain coefficient
        int ee_gain;
        // EE KsTa coefficient
        float KsTa;
        // ct1 default -40
        int ct1 = -40;
        // ct2 default 0
        int ct2 = 0;
        // EE corner temp 3
        int ct3;
        // EE corner temp 4
        int ct4;
        // ksto coefficients
        float ksto1;
        float ksto2;
        float ksto3;
        float ksto4;
        // Sensitivity correction coefficients
        float alpha_corr_range1;
        float alpha_corr_range2;
        float alpha_corr_range3;
        float alpha_corr_range4;
        // alpha corner pixel subpage 0 and 1
        float alpha_cp_sp_0;
        float alpha_cp_sp_1;
        // offset cornerpixel subpage 0 and 1
        int off_cp_sp0;
        int off_cp_sp1;
        // Kv cornerpixel coefficient
        float Kv_cp;
        // Kta cornerpixel coefficient
        float Kta_cp;
        // Resolution control coefficient
        uint16_t resolution_ee;
    };
} // namespace r2d2::thermal_camera
