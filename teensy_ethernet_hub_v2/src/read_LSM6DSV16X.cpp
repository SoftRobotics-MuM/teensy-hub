#include "read_LSM6DSV16X.h"
#include "LSM6DSV16X_regs.h"
#include "MMC5603_regs.h"
#include "sensor_cfg.h"
#include <math.h>

// --- Error-checked direct I2C access ----------------------------------------
// Register auto-increment (CTRL3.IF_INC) is enabled by the library's begin(),
// so multi-byte reads walk consecutive registers.

static bool bus_read_regs(TwoWire& wire, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len){
  wire.beginTransmission(addr);
  wire.write(reg);
  if (wire.endTransmission(false) != 0) {   // address/register phase NACKed
    return false;
  }
  if (wire.requestFrom(addr, len) != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = wire.read();
  }
  return true;
}

static bool bus_write_reg(TwoWire& wire, uint8_t addr, uint8_t reg, uint8_t value){
  wire.beginTransmission(addr);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission(true) == 0;
}

static bool i2c_read_regs(ImuSite& site, uint8_t reg, uint8_t* buf, uint8_t len){
  return bus_read_regs(site.wire, site.i2c_addr, reg, buf, len);
}

static bool i2c_write_reg(ImuSite& site, uint8_t reg, uint8_t value){
  return bus_write_reg(site.wire, site.i2c_addr, reg, value);
}

// MMC5603 output (9 bytes from Xout0) -> 20-bit values centred on zero [LSB].
static void mmc_decode(const uint8_t* d, int32_t* out){
  out[0] = (int32_t)((uint32_t)d[0] << 12 | (uint32_t)d[1] << 4 | (uint32_t)d[6] >> 4) - (1 << 19);
  out[1] = (int32_t)((uint32_t)d[2] << 12 | (uint32_t)d[3] << 4 | (uint32_t)d[7] >> 4) - (1 << 19);
  out[2] = (int32_t)((uint32_t)d[4] << 12 | (uint32_t)d[5] << 4 | (uint32_t)d[8] >> 4) - (1 << 19);
}

static const float MMC_UT_PER_LSB = 0.00625f;

// --- Boot-time SET/RESET offset measurement ------------------------------------
// The MMC5603 is only reachable through the IMU's sensor hub (I2C master);
// pass-through to the primary bus is not possible on this PCB. Single
// register writes/reads are therefore done as one-shot sensor-hub
// transactions, as in ST's lsm6dsv16x_sh_cfg_write()/_read(): configure
// slave 0 (and 1), switch the master on, wait for the transaction to end
// (WR_ONCE_DONE / SENS_HUB_ENDOP), switch the master off. The hub is
// triggered by the accelerometer data-ready (960 Hz here).

// STATUS_MASTER_MAINPAGE (48h) bits, see ST lsm6dsv16x_reg.h
static const uint8_t STATUS_MASTER_SENS_HUB_ENDOP = 1u << 0;
static const uint8_t STATUS_MASTER_SLAVE0_NACK = 1u << 3;
static const uint8_t STATUS_MASTER_SLAVE1_NACK = 1u << 4;
static const uint8_t STATUS_MASTER_WR_ONCE_DONE = 1u << 7;

// MASTER_CONFIG (sensor-hub bank) bits
static const uint8_t MASTER_CONFIG_WRITE_ONCE = 1u << 6;
static const uint8_t MASTER_CONFIG_MASTER_ON = 1u << 2;
static const uint8_t MASTER_CONFIG_AUX_2_SENSORS = 0b01;
// SLVx_CONFIG: SHUB_ODR[2:0] (bits 7:5) = 480 Hz (fastest)
static const uint8_t SLV_CONFIG_SHUB_ODR_480HZ = 0b110 << 5;

static const uint8_t FUNC_CFG_SHUB_BANK = 0b01000000;

// MMC5603 INT_CTRL_0 commands and STATUS_1 bit (datasheet).
static const uint8_t MMC_CMD_TAKE_MEAS_M = 0x01;
static const uint8_t MMC_CMD_DO_SET = 0x08;
static const uint8_t MMC_CMD_DO_RESET = 0x10;
static const uint8_t MMC_STATUS1_MEAS_M_DONE = 0x40;
static const uint8_t MMC5603_PRODUCT_ID_VALUE = 0x10;

// Writes the sensor-hub bank registers given as (reg, value) pairs, then
// switches back to the main bank (also on error).
static bool shub_bank_write(ImuSite& site, const uint8_t (*rv)[2], int n){
  bool ok = i2c_write_reg(site, REG_LSM6DSV16X_FUNC_CFG_ACCESS, FUNC_CFG_SHUB_BANK);
  for (int i = 0; ok && i < n; i++) {
    ok = i2c_write_reg(site, rv[i][0], rv[i][1]);
  }
  return i2c_write_reg(site, REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0x00) && ok;
}

static bool shub_master(ImuSite& site, uint8_t master_config){
  const uint8_t rv[][2] = {{REG_LSM6DSV16X_MASTER_CONFIG, master_config}};
  return shub_bank_write(site, rv, 1);
}

// Waits until all bits of 'mask' are set in STATUS_MASTER_MAINPAGE; fails on
// timeout or a NACK of slave 0/1.
static bool shub_wait(ImuSite& site, uint8_t mask, uint32_t timeout_ms){
  const uint32_t t0 = millis();
  for (;;) {
    uint8_t st = 0;
    if (!i2c_read_regs(site, REG_LSM6DSV16X_STATUS_MASTER_MAINPAGE, &st, 1)) {
      return false;
    }
    if (st & (STATUS_MASTER_SLAVE0_NACK | STATUS_MASTER_SLAVE1_NACK)) {
      return false;
    }
    if ((st & mask) == mask) {
      return true;
    }
    if (millis() - t0 > timeout_ms) {
      return false;
    }
    delayMicroseconds(200);
  }
}

// One register write to the MMC5603 (slave 0, write once).
static bool mmc_hub_write(ImuSite& site, uint8_t reg, uint8_t value){
  const uint8_t cfg[][2] = {
    {REG_LSM6DSV16X_MASTER_CONFIG, MASTER_CONFIG_WRITE_ONCE},          // master off
    {REG_LSM6DSV16X_SLV0_ADD, (uint8_t)(MMC5603_I2C_ADDR << 1 | 0)},   // write
    {REG_LSM6DSV16X_SLV0_SUBADD, reg},
    {REG_LSM6DSV16X_DATAWRITE_SLV0, value},
    {REG_LSM6DSV16X_SLV0_CONFIG, SLV_CONFIG_SHUB_ODR_480HZ},           // numop = 0
  };
  uint8_t dummy;
  bool ok = shub_bank_write(site, cfg, 5);
  ok = ok && i2c_read_regs(site, REG_LSM6DSV16X_STATUS_MASTER_MAINPAGE, &dummy, 1);
  ok = ok && shub_master(site, MASTER_CONFIG_WRITE_ONCE | MASTER_CONFIG_MASTER_ON);
  ok = ok && shub_wait(site, STATUS_MASTER_WR_ONCE_DONE, 20);
  ok = shub_master(site, MASTER_CONFIG_WRITE_ONCE) && ok;               // master off
  delayMicroseconds(300);
  return ok;
}

// Reads len (1..9) consecutive MMC5603 registers from 'reg' (slave 0: up to
// 6 bytes, slave 1: the rest) into buf.
static bool mmc_hub_read(ImuSite& site, uint8_t reg, uint8_t* buf, uint8_t len){
  if (len == 0 || len > 9) {
    return false;
  }
  const uint8_t n0 = len > 6 ? 6 : len;
  const uint8_t n1 = len - n0;
  const uint8_t cfg[][2] = {
    {REG_LSM6DSV16X_MASTER_CONFIG, (uint8_t)(n1 ? MASTER_CONFIG_AUX_2_SENSORS : 0)},   // master off
    {REG_LSM6DSV16X_SLV0_ADD, (uint8_t)(MMC5603_I2C_ADDR << 1 | 1)},                   // read
    {REG_LSM6DSV16X_SLV0_SUBADD, reg},
    {REG_LSM6DSV16X_SLV0_CONFIG, (uint8_t)(SLV_CONFIG_SHUB_ODR_480HZ | n0)},
    {REG_LSM6DSV16X_SLV1_ADD, (uint8_t)(MMC5603_I2C_ADDR << 1 | 1)},
    {REG_LSM6DSV16X_SLV1_SUBADD, (uint8_t)(reg + n0)},
    {REG_LSM6DSV16X_SLV1_CONFIG, n1},
  };
  uint8_t dummy;
  bool ok = shub_bank_write(site, cfg, n1 ? 7 : 4);
  ok = ok && i2c_read_regs(site, REG_LSM6DSV16X_STATUS_MASTER_MAINPAGE, &dummy, 1);
  ok = ok && shub_master(site, (uint8_t)((n1 ? MASTER_CONFIG_AUX_2_SENSORS : 0) | MASTER_CONFIG_MASTER_ON));
  // ENDOP is a level (transaction concluded), not an event: give the hub
  // time for at least one complete cycle after switching on (480 Hz -> ~2 ms).
  delay(5);
  ok = ok && shub_wait(site, STATUS_MASTER_SENS_HUB_ENDOP, 20);
  ok = shub_master(site, (uint8_t)(n1 ? MASTER_CONFIG_AUX_2_SENSORS : 0)) && ok;     // master off
  delayMicroseconds(300);
  ok = ok && i2c_write_reg(site, REG_LSM6DSV16X_FUNC_CFG_ACCESS, FUNC_CFG_SHUB_BANK);
  ok = ok && i2c_read_regs(site, REG_LSM6DSV16X_SENSOR_HUB_1, buf, len);
  ok = i2c_write_reg(site, REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0x00) && ok;
  return ok;
}

// One single measurement (one-shot mode, default bandwidth: ~6.6 ms).
static bool mmc_measure(ImuSite& site, int32_t* out){
  if (!mmc_hub_write(site, REG_MMC5603_INT_CTR_0, MMC_CMD_TAKE_MEAS_M)) {
    return false;
  }
  delay(8);
  uint8_t status = 0;
  for (int i = 0; i < 5; i++) {
    if (!mmc_hub_read(site, REG_MMC5603_STATUS_1, &status, 1)) {
      return false;
    }
    if (status & MMC_STATUS1_MEAS_M_DONE) {
      break;
    }
    delay(2);
  }
  if (!(status & MMC_STATUS1_MEAS_M_DONE)) {
    return false;
  }
  uint8_t d[9];
  if (!mmc_hub_read(site, REG_MMC5603_X_OUT_0, d, sizeof(d))) {
    return false;
  }
  mmc_decode(d, out);
  return true;
}

// SET puts +H on the output, RESET -H; the sensor's bridge offset keeps its
// sign: M_set = H + O, M_reset = -H + O  ->  O = (M_set + M_reset) / 2.
// MAG_SETRESET_PAIRS pairs are averaged (~60 ms per pair). Ends with a SET
// (normal state) and the sensor-hub master off; the continuous-mode
// configuration in setup_LSM6DSV16X() follows. On failure the site's
// mag_offset_valid stays false and nothing else changes.
static void measure_mag_setreset_offset(ImuSite& site){
  site.mag_offset_valid = false;
  uint8_t id = 0;
  bool ok = mmc_hub_read(site, REG_MMC5603_PRODUCT_ID, &id, 1) && id == MMC5603_PRODUCT_ID_VALUE;
  // After a Teensy reset without power cycle the MMC5603 is still in
  // continuous mode from the previous run: stop it (Cmm_en = 0, and
  // INT_CTRL_0 rewritten without Auto_SR_en) and let a running measurement end.
  ok = ok && mmc_hub_write(site, REG_MMC5603_INT_CTR_2, 0x00);
  ok = ok && mmc_hub_write(site, REG_MMC5603_INT_CTR_0, 0x00);
  delay(10);

  double sum_o[3] = {0, 0, 0}, sum_o2[3] = {0, 0, 0}, sum_h[3] = {0, 0, 0};
  int n = 0;
  for (int k = 0; ok && k < MAG_SETRESET_PAIRS; k++) {
    int32_t ms[3], mr[3];
    ok = mmc_hub_write(site, REG_MMC5603_INT_CTR_0, MMC_CMD_DO_SET);
    delay(1);
    ok = ok && mmc_measure(site, ms);
    ok = ok && mmc_hub_write(site, REG_MMC5603_INT_CTR_0, MMC_CMD_DO_RESET);
    delay(1);
    ok = ok && mmc_measure(site, mr);
    if (!ok) {
      break;
    }
    for (int a = 0; a < 3; a++) {
      const double o = 0.5 * (ms[a] + mr[a]) * MMC_UT_PER_LSB;
      sum_o[a] += o;
      sum_o2[a] += o * o;
      sum_h[a] += 0.5 * (ms[a] - mr[a]) * MMC_UT_PER_LSB;
    }
    n++;
  }
  // leave the sensor in the SET state (as after power-up / automatic SET)
  mmc_hub_write(site, REG_MMC5603_INT_CTR_0, MMC_CMD_DO_SET);
  delay(1);
  shub_master(site, 0x00);   // master off, single sensor, write-once off

  if (!ok || n == 0) {
    return;
  }
  float max_std = 0.0f;
  for (int a = 0; a < 3; a++) {
    const double mean = sum_o[a] / n;
    const double var = sum_o2[a] / n - mean * mean;
    site.mag_offset_uT[a] = (float)mean;
    site.mag_setreset_field_uT[a] = (float)(sum_h[a] / n);
    const float sd = var > 0.0 ? (float)sqrt(var) : 0.0f;
    if (sd > max_std) {
      max_std = sd;
    }
  }
  site.mag_offset_std_uT = max_std;
  site.mag_offset_n = (uint8_t)n;
  site.mag_offset_valid = true;
}

bool setup_LSM6DSV16X(ImuSite& site){
  LSM6DSV16XSensor& sensor = site.sensor;

  uint8_t who_am_i = 0;
  if (!i2c_read_regs(site, REG_LSM6DSV16X_WHO_AM_I, &who_am_i, 1) || who_am_i != LSM6DSV16X_ID) {
    return false;
  }

  sensor.begin();
  sensor.Set_X_Power_Mode(LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);
  sensor.Set_G_Power_Mode(LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);
  sensor.Enable_X();
  sensor.Enable_G();

  // Full scale in physical units (g / dps), NOT the lsm6dsv16x_*_full_scale_t
  // enum values: Set_X_FS()/Set_G_FS() round the number up to the next range,
  // so passing the enums (LSM6DSV16X_16g = 3, LSM6DSV16X_4000dps = 12) used to
  // select +-4 g and +-125 dps instead.
  sensor.Set_X_FS(16);    // [g]   2 | 4 | 8 | 16
  sensor.Set_G_FS(4000);  // [dps] 125 | 250 | 500 | 1000 | 2000 | 4000

  sensor.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE); // Don't store IMU data in FIFO, otherwise reading the FIFO takes too long

  sensor.Set_X_ODR(960.0f, LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);  //[Hz] 7.5f | 15.0f | 30.0f | 60.0f | 120.0f | 240.0f | 480.0f | 960.0f | 1920.0f | 3840.0f | 7680.0f 
  sensor.Set_G_ODR(960.0f, LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);  //[Hz] 7.5f | 15.0f | 30.0f | 60.0f | 120.0f | 240.0f | 480.0f | 960.0f | 1920.0f | 3840.0f | 7680.0f 
  // No on-chip sensor fusion (SFLP rotation/gravity/gyro-bias) and no FIFO:
  // only raw accel/gyro/mag are read; orientation is computed on the PC side.

  delay(10);

  // setup magnetometer
  sensor.Write_Reg(REG_LSM6DSV16X_IF_CFG, 0b01000000); // Enables I2C pull ups

  // Sensor offset via SET/RESET, before the sensor hub takes over the bus.
  measure_mag_setreset_offset(site);
  sensor.Write_Reg(REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b01000000); // Enables access to the sensor hub registers

  // set sample rate
  // REG_MMC5603_ODR: [ODR[7:0]] set sample rate to 255 (or 1000 if also (hpower=1 and no automatic set/reset))
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_ODR); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 255);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // calculate counter target value
  // REG_MMC5603_INT_CTR_0: [Cmm_freq_en = true | Auto_st_en = false | Auto_SR_en = false | Do Reset = false | Do Set = false | Start_MDT = reserved | Take_meas_T = false | Take_meas_M = false]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = 0x30|write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_0); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b10000000);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // enable automatic set/reset
  // REG_MMC5603_INT_CTR_0: [Cmm_freq_en = false | Auto_st_en = false | Auto_SR_en = true | Do Reset = false | Do Set = false | Start_MDT = reserved | Take_meas_T = false | Take_meas_M = false]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_0); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b00100000);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // set bandwidth (1.2 ms); limitied by sample rate, automatic set/reset
  // REG_MMC5603_INT_CTR_1: [Sw_reset = false | St_enm = false | St_enp = false | Z-inhibit = z-activr | Y-inhibit = y-active | X-inhibit = x-active | BW[1:0] = 1.2 ms]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_1); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b00000011);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // enable continuous mode
  // REG_MMC5603_INT_CTR_2: [hpower = true (makes 1000 Hz possible if automatic set/reset turned off) | reserved | reserved | Cmm_en = continuous mode | En_prd_se = periodical set enabled | Prd_set[2:0] = set@1/25/sample]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_2); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b10011001);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // cofigure which registers to read
  // Note: as there are too many registers this has to be split "virtually" to two external devices (0 and 1)
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000101); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 2 sensors]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 1); // slave0 address: [7-bit address = MMC5603_I2C_ADDR | read]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV1_ADD, (MMC5603_I2C_ADDR << 1) | 1); // slave1 address: [7-bit address = MMC5603_I2C_ADDR | read]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_X_OUT_0); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_SLV1_SUBADD, REG_MMC5603_X_OUT_2); // slave1 register address
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10100110); // SLV0_CONFIG: [SHUB_ODR_[2:0] = 240 Hz | BATCH_EXT_SENS_0_EN = FIFI batching disabled | Slave0_numop[2:0] = read 6 registers]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV1_CONFIG, 0b00010011); // SLV1_CONFIG: [reserved | reserved | reserved | reserved | BATCH_EXT_SENS_0_EN = FIFI batching disabled | Slave1_numop[2:0] = read 3 registers]
  delay(100);

  sensor.Write_Reg(REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b00000000); // Disables access to the sensor hub registers

  // Raw-LSB -> mg / mdps factors for the full scales configured above, read
  // back from the sensor (same values the library's Get_X_Axes()/Get_G_Axes()
  // would use).
  if (sensor.Get_X_Sensitivity(&site.acc_mg_per_lsb) != LSM6DSV16X_OK ||
      sensor.Get_G_Sensitivity(&site.gyro_mdps_per_lsb) != LSM6DSV16X_OK ||
      site.acc_mg_per_lsb == 0.0f || site.gyro_mdps_per_lsb == 0.0f) {
    return false;
  }

  // Final presence check: the IMU still answers after configuration.
  who_am_i = 0;
  return i2c_read_regs(site, REG_LSM6DSV16X_WHO_AM_I, &who_am_i, 1) && who_am_i == LSM6DSV16X_ID;
}

bool read_magnetometer(ImuSite& site, double* mag_field){
  // Sensor-hub status (main page): did the IMU's I2C master get a NACK from
  // the magnetometer (slave 0 = Xout0..Zout1, slave 1 = Xout2..Zout2)?
  uint8_t master_status = 0;
  if (!i2c_read_regs(site, REG_LSM6DSV16X_STATUS_MASTER_MAINPAGE, &master_status, 1)) {
    return false;
  }
  if (master_status & (STATUS_MASTER_SLAVE0_NACK | STATUS_MASTER_SLAVE1_NACK)) {
    return false;
  }

  // read sensor: switch to the sensor-hub register bank, one 9-byte burst
  // (SENSOR_HUB_1..9), switch back. The switch back is attempted even if the
  // burst failed, so the IMU is never left in the sensor-hub bank.
  uint8_t data_ext[9];
  bool ok = i2c_write_reg(site, REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b01000000); // Enables access to the sensor hub
  ok = ok && i2c_read_regs(site, REG_LSM6DSV16X_SENSOR_HUB_1, data_ext, sizeof(data_ext));
  ok = i2c_write_reg(site, REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b00000000) && ok; // Disables access to the sensor hub
  if (!ok) {
    return false;
  }

  // 20-bit raw values centred on zero, scaled to uT (LSB from the datasheet)
  int32_t raw[3];
  mmc_decode(data_ext, raw);
  for (int a = 0; a < 3; a++) {
    mag_field[a] = (double)raw[a] * MMC_UT_PER_LSB;
#if MAG_SUBTRACT_SETRESET_OFFSET
    if (site.mag_offset_valid) {
      mag_field[a] -= site.mag_offset_uT[a];
    }
#endif
  }
  return true;
}

bool read_imu(ImuSite& site, float* accel_mg, float* angrate_mdps){
  // One burst over OUTX_L_G..OUTZ_H_A (22h..2Dh): gyro X,Y,Z then accel X,Y,Z,
  // little-endian int16 each (BDU is enabled by the library's begin()).
  uint8_t raw[12];
  if (!i2c_read_regs(site, REG_LSM6DSV16X_OUTX_L_G, raw, sizeof(raw))) {
    return false;
  }
  for (int axis = 0; axis < 3; axis++) {
    const int16_t g = (int16_t)((uint16_t)raw[2 * axis + 1] << 8 | raw[2 * axis]);
    const int16_t a = (int16_t)((uint16_t)raw[6 + 2 * axis + 1] << 8 | raw[6 + 2 * axis]);
    angrate_mdps[axis] = g * site.gyro_mdps_per_lsb;
    accel_mg[axis] = a * site.acc_mg_per_lsb;
  }
  return true;
}
