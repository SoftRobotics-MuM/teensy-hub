#ifndef sensor_cfg_h
#define sensor_cfg_h
#include <stdint.h>

// Selects which sensor hardware generation is compiled in. There is never a
// mix of the two on one robot, so this is a single compile-time switch, not
// a runtime setting - flip it and reflash for a hardware swap.
//   0 = old hardware: 4 independent I2C LSM6DSV16X+MMC5603 sites
//       (read_LSM6DSV16X.cpp/.h, LSM6DSV16X_regs.h, MMC5603_regs.h)
//   1 = new hardware: sensor-pcb-v2, up to 8 boards sharing one SPI bus,
//       each board carrying 2x LSM6DSV16BX + 2x MMC5633NJL relayed through
//       an STM32 (sensor_pcb_v2.cpp/.h, Lsm6dsv16bx.cpp/.h)
#define USE_NEW_SENSOR_PCB 0

#if USE_NEW_SENSOR_PCB

#include <Arduino.h>

// --- Bus pin configuration (Teensy 4.1 SPI1) --------------------------------
// Confirmed against this project's actual wiring (not the master test
// harness's default hardware-SPI0 pins, which collide with servos 9-12 on
// this board - see servo_cfg.h's PIN_MOTOR_9..12).
constexpr uint8_t PIN_BUS_CS   = 38;
constexpr uint8_t PIN_BUS_SCK  = 27;
constexpr uint8_t PIN_BUS_MISO = 39;
constexpr uint8_t PIN_BUS_MOSI = 26;

// SPI clock for the entire shared bus (STM32 command channel + direct IMU
// access - same physical wires). Field-verified reliable at 10 MHz on the
// master test harness with solid wiring; lower this if you see intermittent
// failures, especially after rewiring.
constexpr uint32_t SPI_BUS_HZ = 20000000;

// --- Shared-bus command-channel timing --------------------------------------
// Dictated by the STM32 firmware's own ISR processing time, not something to
// trade off freely - the STM32 boards are potted and cannot be reflashed in
// the field. Its own measured worst case is ~30us; T_PROC_GUARD_US keeps a
// large safety margin above that.
constexpr uint32_t T_PROC_GUARD_US = 150;
constexpr uint32_t T_CS_SETUP_GUARD_US = 15;

// --- Board population --------------------------------------------------------
constexpr uint8_t MAX_SENSOR_BOARDS = 8;

// Which board addresses (0..7) are physically present on the bus right now.
// One board today; extend this as more boards are added - nothing else here
// needs to change to support up to MAX_SENSOR_BOARDS.
constexpr bool BOARD_ENABLED[MAX_SENSOR_BOARDS] = {
  true,   // 0
  false,  // 1
  false,  // 2
  false,  // 3
  false,  // 4
  false,  // 5
  false,  // 6
  false,  // 7
};

#endif // USE_NEW_SENSOR_PCB

#if !USE_NEW_SENSOR_PCB
// I2C clock of Wire1/Wire2 (IMUs). The MMC5603 sits on each IMU's auxiliary
// bus and is only reachable through the IMU's sensor hub (no pass-through on
// this PCB).
constexpr uint32_t IMU_I2C_CLOCK_HZ = 1000000;

// --- MMC5603 boot-time SET/RESET offset (old hardware) ----------------------
// At startup every site measures its magnetometer's own zero-field offset
// with SET/RESET pairs (one-shot sensor-hub transactions, before continuous
// mode starts): offset = (M_set + M_reset) / 2. ~60 ms per pair and site.
#define MAG_SETRESET_PAIRS 8
// 1: subtract that offset from every magnetometer reading (mag1).
// 0: only measure and report it (Serial at boot / on agent connect, and in
//    SensorBoard.mag2 if MAG_PUBLISH_SETRESET_OFFSET) - for checking whether
//    the offset is significant and stable across power cycles first.
#define MAG_SUBTRACT_SETRESET_OFFSET 1
// 1: publish the measured offset [T] in SensorBoard.mag2 (unused on the old
//    hardware). STATUS_MAG2_VALID stays clear, so consumers that check the
//    status bits ignore it; it is recorded in every bag.
#define MAG_PUBLISH_SETRESET_OFFSET 1
#endif

// --- Physical-unit conversion (shared by both hardware generations) --------
// Kept on the Teensy, in one place, deliberately: sensor settings (ODR, full
// scale) are also set on the Teensy, so changing either stays a one-file,
// one-rebuild change instead of a Teensy+PC pair that has to be kept in sync
// by hand.

constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;

// mg -> m/s^2 and mdps -> rad/s: unit-only conversions, valid regardless of
// which full-scale range produced the value - both hardware paths reduce to
// "milli-g"/"milli-degrees-per-second" before this step (old hardware's
// vendor library returns mg/mdps directly already scaled for its configured
// FS; new hardware's raw LSB counts are first scaled to mg/mdps below).
constexpr float MG_TO_MPS2   = STANDARD_GRAVITY_MPS2 / 1000.0f;
constexpr float MDPS_TO_RADPS = 0.017453293f /*pi/180*/ / 1000.0f;

#if USE_NEW_SENSOR_PCB
// LSM6DSV16BX raw-LSB -> mg/mdps, at this part's power-on-reset full scale
// (the master driver's configure() never touches CTRL8_XL/CTRL6_G, so the
// sensor stays at reset default: FS_XL=+-2g, FS_G=+-125dps). Verified
// against ST's own lsm6dsv16bx-pid driver (lsm6dsv16bx_reg.c):
// lsm6dsv16bx_from_fs2_to_mg() = lsb*0.061, lsm6dsv16bx_from_fs125_to_mdps()
// = lsb*4.375. Re-derive these if CTRL8_XL/CTRL6_G are ever configured to a
// different full scale.
constexpr float ACCEL_LSB_TO_MG   = 0.061f;
constexpr float GYRO_LSB_TO_MDPS  = 4.375f;
#endif

// MMC5603/MMC5633NJL: 0.00625 uT per LSB of the reconstructed 20-bit raw
// value (same family, same sensitivity spec, same interleaved register
// layout - both this project's old mag-read code and the new protocol's
// CMD_GET_MAG response reconstruct the identical 20-bit value).
constexpr float MAG_LSB_TO_TESLA = 0.00625e-6f;

#endif
