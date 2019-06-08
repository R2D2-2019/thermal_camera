#pragma once

/**
 * MLX90640 16 bits registers
 * INTERNAL means internal register.
 * EE means EEPROM (Electric Erasable Programmable Read-Only)
 * RAM means just RAM (Random Acces Memory)
 */
enum class registers : uint16_t {
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