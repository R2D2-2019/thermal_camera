#pragma once

/**
 * MLX90640 16 bits registers
 * INTERNAL means internal register.
 * EE means EEPROM (Electric Erasable Programmable Read-Only)
 * RAM means just RAM (Random Acces Memory)
 **/
enum class registers : uint16_t {
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