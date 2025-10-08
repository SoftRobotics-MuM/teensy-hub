#include <stdint.h>

#ifndef MMC5603_REGS_H
#define MMC5603_REGS_H

const uint8_t MMC5603_I2C_ADDR    = 0x30;    // MMC5603 7-bit address

// MMC5603 registers
const uint8_t REG_MMC5603_X_OUT_0 = 0x00;
const uint8_t REG_MMC5603_X_OUT_1 = 0x01;
const uint8_t REG_MMC5603_Y_OUT_0 = 0x02;
const uint8_t REG_MMC5603_Y_OUT_1 = 0x03;
const uint8_t REG_MMC5603_Z_OUT_0 = 0x04;
const uint8_t REG_MMC5603_Z_OUT_1 = 0x05;
const uint8_t REG_MMC5603_X_OUT_2 = 0x06;
const uint8_t REG_MMC5603_Y_OUT_2 = 0x07;
const uint8_t REG_MMC5603_Z_OUT_2 = 0x08;
const uint8_t REG_MMC5603_T_OUT = 0x09; // Temperature output

const uint8_t REG_MMC5603_STATUS_1 = 0x18;

const uint8_t REG_MMC5603_ODR = 0x1A;
const uint8_t REG_MMC5603_INT_CTR_0 = 0x1B;
const uint8_t REG_MMC5603_INT_CTR_1 = 0x1C;
const uint8_t REG_MMC5603_INT_CTR_2 = 0x1D;

const uint8_t REG_MMC5603_ST_X_TH = 0x1E;
const uint8_t REG_MMC5603_ST_Y_TH = 0x1F;
const uint8_t REG_MMC5603_ST_Z_TH = 0x20;
const uint8_t REG_MMC5603_ST_X = 0x27;
const uint8_t REG_MMC5603_ST_Y = 0x28;
const uint8_t REG_MMC5603_ST_Z = 0x29;

const uint8_t REG_MMC5603_PRODUCT_ID = 0x39;  // Who am I (=0x10)

#endif