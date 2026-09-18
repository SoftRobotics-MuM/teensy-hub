#include "Lsm6dsv16bx.h"
#include "sensor_cfg.h"
#include <string.h>

namespace Lsm6dsv16bx {

// Speed comes from sensor_cfg.h's SPI_BUS_HZ, since this is the same
// physical bus as the STM32 command channel (sensor_pcb_v2.cpp's
// busSettings) - see the note there.
SPISettings spiSettings(SPI_BUS_HZ, MSBFIRST, SPI_MODE0);

void writeReg(SelectFn select, void *ctx, uint8_t reg, uint8_t value) {
    select(ctx, true);
    SPI1.beginTransaction(spiSettings);
    uint8_t buf[2] = { (uint8_t)(reg & 0x7F), value };
    SPI1.transfer(buf, sizeof(buf));
    SPI1.endTransaction();
    select(ctx, false);
}

uint8_t readReg(SelectFn select, void *ctx, uint8_t reg) {
    select(ctx, true);
    SPI1.beginTransaction(spiSettings);
    uint8_t buf[2] = { (uint8_t)(0x80 | reg), 0x00 };
    SPI1.transfer(buf, sizeof(buf));
    SPI1.endTransaction();
    select(ctx, false);
    return buf[1];
}

void readRegs(SelectFn select, void *ctx, uint8_t reg, uint8_t *dst, uint8_t len) {
    select(ctx, true);
    SPI1.beginTransaction(spiSettings);
    uint8_t buf[1 + 12];  // 12 is the largest block readRaw() reads at once
    buf[0] = (uint8_t)(0x80 | reg);
    memset(&buf[1], 0, len);
    SPI1.transfer(buf, (size_t)(1 + len));
    memcpy(dst, &buf[1], len);
    SPI1.endTransaction();
    select(ctx, false);
}

void writeRegs(SelectFn select, void *ctx, uint8_t reg, const uint8_t *src, uint8_t len) {
    select(ctx, true);
    SPI1.beginTransaction(spiSettings);
    uint8_t buf[1 + 2];  // 2 is the largest burst configure() writes at once
    buf[0] = (uint8_t)(reg & 0x7F);
    memcpy(&buf[1], src, len);
    SPI1.transfer(buf, (size_t)(1 + len));
    SPI1.endTransaction();
    select(ctx, false);
}

void configure(SelectFn select, void *ctx) {
    // Must land first, alone: establishes IF_INC=1 before the burst
    // below begins - see the note on writeRegs()/configure() in
    // Lsm6dsv16bx.h for why this can't just be folded into one 3-byte
    // burst instead.
    writeReg(select, ctx, CTRL3, CTRL3_BDU_IFINC);

    uint8_t ctrl1AndCtrl2[2] = { CTRL1_XL_120HZ_HIGHPERF, CTRL2_G_120HZ_HIGHPERF };
    writeRegs(select, ctx, CTRL1, ctrl1AndCtrl2, sizeof(ctrl1AndCtrl2));
}

static void unpackRaw(const uint8_t *rawBlock, Raw &out) {
    out.gx = (int16_t)((rawBlock[1] << 8) | rawBlock[0]);
    out.gy = (int16_t)((rawBlock[3] << 8) | rawBlock[2]);
    out.gz = (int16_t)((rawBlock[5] << 8) | rawBlock[4]);
    // Accelerometer registers are Z,Y,X order on this part - see the
    // block comment in Lsm6dsv16bx.h.
    out.az = (int16_t)((rawBlock[7]  << 8) | rawBlock[6]);
    out.ay = (int16_t)((rawBlock[9]  << 8) | rawBlock[8]);
    out.ax = (int16_t)((rawBlock[11] << 8) | rawBlock[10]);
}

bool readRaw(SelectFn select, void *ctx, Raw &out) {
    uint8_t whoAmI = readReg(select, ctx, WHO_AM_I_REG);
    configure(select, ctx);

    uint8_t raw[12];
    readRegs(select, ctx, OUTX_L_G, raw, sizeof(raw));
    unpackRaw(raw, out);

    return whoAmI == WHO_AM_I_ID;
}

void readData(SelectFn select, void *ctx, Raw &out) {
    uint8_t raw[12];
    readRegs(select, ctx, OUTX_L_G, raw, sizeof(raw));
    unpackRaw(raw, out);
}

}  // namespace Lsm6dsv16bx
