// =============================================================================
// Protocol.h - Shared-SPI-bus command protocol between the Teensy 4.1 (master)
//              and up to 8 STM32C011 boards (slaves), sharing SCK/MOSI/MISO/NSS.
//
// Vendored VERBATIM from sensor-pcb-v2-slave-software's Core/Inc/Protocol.h,
// which remains THE CANONICAL SOURCE - this file is pure C, no HAL
// dependency, so unlike the master test harness's hand-mirrored .ino enum,
// this project includes it directly instead of re-declaring the same
// commands/constants a second time. When updating, replace this file
// wholesale from the slave repo rather than hand-editing it here, so it
// never drifts from the canonical copy.
//
// BUS TOPOLOGY
// ------------
// PA4(NSS)/PA5(SCK)/PA6(MISO)/PA7(MOSI) of every board, AND the two
// LSM6DSV16BX IMUs on every board, are all wired in PARALLEL to the same 4
// signals on the Teensy. The Teensy is the ONLY SPI master on this bus,
// always. The STM32 NEVER becomes an SPI master and never talks to its own
// IMUs directly - its only job regarding the IMUs is to assert/deassert
// their local chip-select lines (PA8 = IMU1 CS, PA11 = IMU2 CS) on command.
// The Teensy runs the LSM6DSV16BX register-level driver itself and reads
// the IMUs' MISO output directly, exactly like any normal SPI peripheral,
// once the correct board has pulled the correct CS low.
//
// This keeps exactly one SPI master on the bus at all times, which is what
// makes collision avoidance tractable:
//
//   * Every board's MISO pin defaults to a plain GPIO INPUT (high-Z). It is
//     only switched into SPI-AF push-pull OUTPUT mode for the exact byte
//     window in which THAT board, and only that board, is answering an
//     addressed STM32 command (mag data, ping, etc).
//   * PA4 (NSS) MUST be held HIGH by the Teensy for the entire duration of
//     any direct Teensy<->IMU transaction. With NSS high, every STM32's
//     SPI1 peripheral is hardware-deselected: it does not clock, does not
//     interrupt, and its MISO stays tri-stated - so it is simply invisible
//     to the bus while an IMU is being read directly. NSS only goes low
//     when the Teensy is running a command/response exchange with a
//     specific STM32 (mag data, LED, or "set my IMU CS").
//   * Only ONE chip-select - across all boards and both IMUs on each board
//     - may ever be asserted low at the same time, since they all share
//     MISO. The STM32 enforces mutual exclusion of its OWN two CS lines,
//     but it is the TEENSY's responsibility to never select a second
//     board/IMU before deselecting the previous one.
//
// TWO-PHASE TRANSACTION MODEL (for the STM32 command channel only - direct
// IMU access is just a normal single SPI transaction with NSS high, see
// above)
// ----------------------------
// Every STM32 command is a separate NSS-framed SPI transaction, split into:
//
//   Phase A - COMMAND (always exactly 4 bytes, NSS asserted then released):
//       byte0 = CMD_*             (see enum below)
//       byte1 = target address    (0..7, or ADDR_BROADCAST = 0xFF)
//       byte2 = data0             (command-specific, 0 if unused)
//       byte3 = data1             (command-specific, 0 if unused)
//
//     ... Teensy MUST wait >= T_PROC_GUARD_US (recommended 20 us) with NSS
//     high before starting Phase B (if any) or a direct IMU transaction,
//     to give the STM32's ISR time to act on the command ...
//
//   Phase B - DATA (only for commands that produce a response; NSS asserted
//     again, Teensy clocks out N dummy 0x00 bytes and reads N response
//     bytes back). Only the addressed board drives MISO during Phase B;
//     every other board's MISO stays high-Z the whole time.
//
// SPI mode: Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit words - used for BOTH
// the STM32 command channel and direct IMU access (the LSM6DSV16BX supports
// mode 0), so the Teensy can use one single SPISettings for the whole bus.
//
// =============================================================================
#pragma once
#include <stdint.h>

#define ADDR_BROADCAST   0xFF

// --- Commands (Phase-A byte0) ------------------------------------------------
// NOTE: was `enum : uint8_t { ... }` in the Arduino/C++ version - a fixed
// underlying-type enum is a C++ feature (and a very recent, not-yet-widely-
// supported C23 one); plain arm-none-eabi-gcc compiling this as C rejects
// that syntax. A plain enum works identically here since every value is
// always stored into a uint8_t buffer byte anyway - the fixed size was
// documentation, not a functional requirement.
enum {
    CMD_PING          = 0x01,  // data0/1 unused. Resp (Phase B, 2 bytes): [0xA5][FW_VERSION]
    CMD_GET_MAG       = 0x02,  // data0/1 unused. Resp: MAG_RESP_LEN bytes, see below.
    CMD_SET_LED       = 0x10,  // data0 = led_mask (bit0=LED1,bit1=LED2)
                                 // data1 = led_state (bit0=LED1 on/off, bit1=LED2 on/off)
                                 // No response phase.
    CMD_SET_IMU_CS    = 0x21,  // data0 = which (0 = release both / idle,
                                 //                1 = assert IMU1 CS (PA8),
                                 //                2 = assert IMU2 CS (PA11))
                                 // data1 unused. No response phase. The STM32
                                 // guarantees the OTHER local CS is deasserted
                                 // whenever this is processed (mutual exclusion
                                 // of its own 2 IMUs) - it is the Teensy's job
                                 // to make sure no OTHER board has a CS
                                 // asserted before/while this one does.
                                 //
                                 // addr = ADDR_BROADCAST, data0 = IMU_CS_NONE
                                 //   is a defined "panic release all" - every
                                 //   board releases both its CS lines. Always
                                 //   safe; useful at startup/recovery.
                                 // addr = ADDR_BROADCAST with data0 != NONE
                                 //   is UNDEFINED/ignored - it would select
                                 //   the same IMU index on every board at
                                 //   once, guaranteed collision. Never send
                                 //   this.
                                 //
                                 // SAFETY INTERLOCK: because every board's
                                 // SPI1 receives every Phase-A frame
                                 // regardless of target address (inherent to
                                 // a shared MOSI/SCK bus), every board also
                                 // snoops CMD_SET_IMU_CS frames addressed to
                                 // OTHER boards. If a board still believes
                                 // its own CS is asserted when it snoops
                                 // another board being told to assert a CS,
                                 // it immediately releases its own CS. This
                                 // is a defense-in-depth safety net for a
                                 // Teensy that forgets to deselect before
                                 // selecting elsewhere - it is NOT a
                                 // substitute for correct select/deselect
                                 // sequencing on the Teensy side (there is
                                 // still a brief window where two drivers
                                 // could contend before the interlock fires).
    CMD_MAG_WRITE_REG = 0x30,  // Writes one raw register on ONE board-local
                                 // MMC5633NJL. Lets the Teensy configure
                                 // mag1/mag2 independently at runtime instead
                                 // of only via this board's compiled-in
                                 // Config.h defaults (MMC5633_ODR_CODE/
                                 // MMC5633_BW_CODE, applied identically to
                                 // both sensors at boot).
                                 // data0 = (target << 7) | reg_addr
                                 //   target: see MAG_WRITE_TARGET_* below.
                                 //   Only 1 bit, not a whole selector byte -
                                 //   the sensor's real register range
                                 //   (0x00..0x1D) fits comfortably in the
                                 //   low 7 bits, so this fits both target
                                 //   and address into data0 and leaves data1
                                 //   free for the value.
                                 // data1 = value to write, verbatim.
                                 // No response phase - fire-and-forget, same
                                 // as CMD_SET_LED. The master is trusted to
                                 // know the MMC5633NJL register map (see the
                                 // datasheet); nothing here validates
                                 // reg_addr or value, and there is no way to
                                 // read back whether the I2C write actually
                                 // succeeded (use CMD_GET_MAG's status byte
                                 // to notice a sensor going bad afterwards).
                                 // addr = ADDR_BROADCAST applies the same
                                 // write to every board's same-selected
                                 // sensor - only useful if every board
                                 // should end up identically configured.
    CMD_MAG_SET_RESET = 0x31,  // Pulses the sensor's SET or RESET coil
                                 // operation (periodic degaussing / offset
                                 // cancellation against the sensor's own
                                 // permanent-magnet bias - see the
                                 // MMC5633NJL datasheet). Unlike
                                 // CMD_MAG_WRITE_REG this gets a whole
                                 // selector byte, so BOTH sensors at once is
                                 // available alongside either one alone.
                                 // data0 = target, see MAG_SETRESET_TARGET_*
                                 //   below.
                                 // data1 = operation, see MAG_OP_* below.
                                 // No response phase. addr = ADDR_BROADCAST
                                 // applies to every board.
};

// --- Response payload layouts ------------------------------------------------
// CMD_GET_MAG response (MAG_RESP_LEN bytes):
//   [0]      board status byte (bit0 = mag1 data valid, bit1 = mag2 data valid)
//   [1..9]   mag1 raw registers 0x00..0x08 as read from the MMC5633NJL, i.e.
//            Xout0,Xout1,Yout0,Yout1,Zout0,Zout1,Xout2,Yout2,Zout2
//            (NOTE the interleaved order - it mirrors the sensor's own
//             register map so no reordering happens in firmware. To
//             reconstruct 20-bit unsigned values on the master:
//               X20 = (Xout0<<12) | (Xout1<<4) | (Xout2 & 0x0F)
//               Y20 = (Yout0<<12) | (Yout1<<4) | (Yout2 & 0x0F)
//               Z20 = (Zout0<<12) | (Zout1<<4) | (Zout2 & 0x0F)
//   [10..18] mag2 raw registers, same layout as mag1
//   [19]     rolling sequence counter (increments every mag cache refresh)
#define MAG_RESP_LEN   20

#define PING_RESP_LEN   2
#define FW_VERSION      0x02   // bumped for CMD_MAG_WRITE_REG/CMD_MAG_SET_RESET

// --- IMU chip-select selector values (CMD_SET_IMU_CS data0) -----------------
#define IMU_CS_NONE   0
#define IMU_CS_IMU1   1
#define IMU_CS_IMU2   2

// --- Magnetometer register-write target (CMD_MAG_WRITE_REG data0 bit 7) ----
#define MAG_WRITE_TARGET_MAG1   0
#define MAG_WRITE_TARGET_MAG2   1

// --- Magnetometer SET/RESET target (CMD_MAG_SET_RESET data0) ---------------
#define MAG_SETRESET_TARGET_MAG1   1
#define MAG_SETRESET_TARGET_MAG2   2
#define MAG_SETRESET_TARGET_BOTH   3

// --- Magnetometer SET/RESET operation (CMD_MAG_SET_RESET data1) ------------
#define MAG_OP_RESET   0
#define MAG_OP_SET     1
