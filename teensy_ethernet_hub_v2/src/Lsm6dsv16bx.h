// =============================================================================
// Lsm6dsv16bx.h - LSM6DSV16BX 6-axis IMU (accel+gyro), raw SPI register
// access only (raw counts, no unit conversion - full scale is left at
// reset default; see sensor_cfg.h for the LSB->physical-unit conversion
// applied by the caller).
//
// Ported from sensor-pcb-v2-master-test-software (SoftRobotics-MuM), with
// the SPI bus and its clock speed changed from the default hardware SPI0
// to SPI1 (see sensor_cfg.h - SPI0's default pins collide with this
// project's servo PWM outputs).
//
// CHIP-SELECT MODEL - read this before using the functions below.
// This sensor's SPI framing only recognizes the very first byte after a
// CS *falling edge* as a register address; every byte clocked in after
// that, for as long as CS stays low, is just data for whatever address
// was already established (auto-incrementing across a multi-byte block
// if IF_INC is set) - never a new address. That means CS must toggle
// around EACH individual register access below; holding it low across
// several back-to-back accesses silently scrambles all but the first
// one (each subsequent "address byte" gets consumed as data instead).
//
// This driver never touches CS itself - every function takes a
// SelectFn callback and invokes it (true, then false) around each
// individual register transaction. That's what keeps this file
// reusable: on a directly-wired sensor, the callback is just
// digitalWrite(csPin, select ? LOW : HIGH); on THIS project it instead
// relays through a remote STM32 board (setImuCs()/CMD_SET_IMU_CS in
// sensor_pcb_v2.cpp, see Protocol.h's bus topology notes) - either way,
// this driver doesn't need to know or care, as long as the callback has
// fully finished asserting/releasing CS by the time it returns, and
// does not itself leave an SPI transaction open (this driver opens its
// own around each access - see the note on SelectFn below for why that
// matters).
//
// Register map/values verified directly against ST's official
// lsm6dsv16bx-pid driver (github.com/STMicroelectronics/lsm6dsv16bx-pid,
// lsm6dsv16bx_reg.h) - NOT the plain LSM6DSV16X's, which differs in two
// places that would otherwise silently misidentify the chip or swap
// axes: WHO_AM_I is 0x71 here (0x70 on the X part), and the
// accelerometer's raw output registers are in Z,Y,X order (0x28=OUTZ_L_A
// ... 0x2C=OUTX_L_A), not the X,Y,Z order the gyro uses and that the X
// part uses for both.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <SPI.h>

namespace Lsm6dsv16bx {

constexpr uint8_t WHO_AM_I_REG = 0x0F;
constexpr uint8_t WHO_AM_I_ID  = 0x71;

constexpr uint8_t CTRL1 = 0x10;  // odr_xl[3:0], op_mode_xl[6:4]
constexpr uint8_t CTRL2 = 0x11;  // odr_g[3:0],  op_mode_g[6:4]
constexpr uint8_t CTRL3 = 0x12;  // if_inc = bit2, bdu = bit6

// 120 Hz, high-performance mode (op_mode=0) on both accel and gyro - any
// non-zero ODR works to get the sensor out of power-down; change here
// for a different rate. Full scale is left untouched (reset default:
// FS_XL=+-2g, FS_G=+-125dps) - see sensor_cfg.h for the matching
// LSB->physical conversion constants.
constexpr uint8_t CTRL1_XL_120HZ_HIGHPERF = 0x06;
constexpr uint8_t CTRL2_G_120HZ_HIGHPERF  = 0x06;
// BDU=1 (block data update - a multi-byte read can't tear across a
// sample boundary), IF_INC=1 (required for readRaw()'s sequential
// 12-byte read; also this part's reset default, set explicitly anyway
// rather than depend on it).
constexpr uint8_t CTRL3_BDU_IFINC = 0x44;

// Gyro block start (X,Y,Z order); IF_INC makes the whole 12-byte
// gyro+accel block sequential from here.
constexpr uint8_t OUTX_L_G = 0x22;

struct Raw {
    int16_t gx, gy, gz;  // raw angular rate, LSB units per CTRL6's FS_G (left at reset default)
    int16_t ax, ay, az;  // raw linear acceleration, LSB units per CTRL8's FS_XL (left at reset default)
};

// Asserts (select=true) or releases (select=false) this sensor's CS.
// 'ctx' is an opaque pointer passed straight through from whatever call
// into this driver triggered it - use it to carry whatever the callback
// needs (a pin number, a board+IMU address for a relay scheme, etc.)
// without this driver needing to know its shape. MUST fully complete
// the assert/release before returning, and must not leave an SPI
// transaction open when it does - this driver opens its own
// (spiSettings, below) around each register access, and Arduino's SPI
// transaction API is not safe to nest.
using SelectFn = void (*)(void *ctx, bool select);

// SPI bus parameters this sensor requires (Mode 0 is mandatory - this
// part doesn't support the others). Speed is defined in Lsm6dsv16bx.cpp
// from sensor_cfg.h's SPI_BUS_HZ.
extern SPISettings spiSettings;

// --- Low-level register access ---------------------------------------
// Standard ST MEMS SPI convention: address byte with bit7 set = read,
// clear = write. Each call is one complete, self-contained CS-bracketed
// transaction (select -> SPI transfer -> deselect) - see the chip-select
// model note above for why that granularity matters.
void writeReg(SelectFn select, void *ctx, uint8_t reg, uint8_t value);
uint8_t readReg(SelectFn select, void *ctx, uint8_t reg);
void readRegs(SelectFn select, void *ctx, uint8_t reg, uint8_t *dst, uint8_t len);

// Burst write starting at 'reg', auto-incrementing through len
// contiguous registers - one CS-bracketed transaction total instead of
// one per register. Only valid where IF_INC is already enabled (see
// configure()'s use of this - it relies on having just set IF_INC=1 via
// its own prior writeReg(CTRL3, ...) call) and only across registers
// that are genuinely contiguous addresses; verified directly against
// ST's own lsm6dsv16bx-pid driver, which uses exactly this pattern
// (lsm6dsv16bx_write_reg(ctx, CTRL1, conf_saved, 2) to restore CTRL1+
// CTRL2 together).
void writeRegs(SelectFn select, void *ctx, uint8_t reg, const uint8_t *src, uint8_t len);

// Writes CTRL3 (BDU+IF_INC) alone, then CTRL1+CTRL2 (ODR+mode for accel
// and gyro) as one auto-incrementing burst - two CS-bracketed
// transactions rather than three. CTRL3 must land first, in its own
// transaction: IF_INC has to already be set before the CTRL1/CTRL2
// burst begins, since it's read once at the start of a transaction, not
// re-checked mid-burst - folding all three into a single 3-byte burst
// would depend on this part's power-on IF_INC default being 1, which
// isn't verified here, so it's deliberately not done that way.
// Idempotent - safe to call on every read if you'd rather not track
// "did I already configure this sensor" state yourself.
void configure(SelectFn select, void *ctx);

// Configures the sensor (see configure()) and reads the raw 12-byte
// gyro+accel block into 'out'. Returns false if WHO_AM_I didn't match
// this part (wiring/CS issue, or a different sensor entirely) - 'out'
// is still populated regardless, since the raw bytes the bus actually
// produced are often worth seeing anyway (e.g. while debugging).
//
// This is the safe, foolproof, but relatively expensive way to read:
// every call is 4 CS-bracketed transactions (WHO_AM_I, CTRL3, CTRL1+
// CTRL2, the data block) - each of which costs a full select()/deselect()
// round trip, potentially expensive depending on what select() actually
// does (on this project, a full STM32 relay command each way). For
// repeated/looped polling once you've confirmed the sensor is present
// and configured, see readData() below instead - it skips straight to
// the one transaction that actually changes between polls.
bool readRaw(SelectFn select, void *ctx, Raw &out);

// Reads ONLY the raw 12-byte gyro+accel block - one CS-bracketed
// transaction, versus readRaw()'s four. Does NOT verify WHO_AM_I or
// (re)configure the sensor - call configure() yourself at least once
// first (typically via one readRaw() call to also confirm the sensor is
// actually present, then readData() for every poll after that). Safe to
// call repeatedly without ever calling configure() again in between, as
// long as the sensor doesn't lose power or get reset independently of
// your own code - CTRL1/2/3 persist in the sensor itself between reads.
void readData(SelectFn select, void *ctx, Raw &out);

}  // namespace Lsm6dsv16bx
