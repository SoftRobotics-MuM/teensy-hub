#ifndef read_LSM6DSV16X_h
#define read_LSM6DSV16X_h

#include <Wire.h>
#include <LSM6DSV16XSensor.h>

// One old-hardware sensor site: an LSM6DSV16X IMU with an MMC5603
// magnetometer attached to its sensor hub (I2C master).
//
// 'sensor' is a reference to the library object (never a copy: the library
// stores 'this' in its register context). 'wire' and 'i2c_addr' (7-bit) are
// used for the error-checked direct I2C reads below - the STM32duino
// library's own I2C helpers ignore bus errors and always report success, so
// its return codes cannot tell whether a sensor actually answered.
struct ImuSite {
  LSM6DSV16XSensor &sensor;
  TwoWire &wire;
  uint8_t i2c_addr;              // 7-bit I2C address
  bool present = false;          // set by setup_LSM6DSV16X()
  float acc_mg_per_lsb = 0.0f;   // read back from the sensor in setup_LSM6DSV16X()
  float gyro_mdps_per_lsb = 0.0f;

  // Boot-time SET/RESET measurement of the MMC5603 (see
  // measure_mag_setreset_offset() in read_LSM6DSV16X.cpp):
  //   offset = (M_set + M_reset) / 2   sensor's own zero-field offset
  //   field  = (M_set - M_reset) / 2   field without the sensor offset
  // Hard-iron offsets of parts near the sensor are part of 'field', not of
  // 'offset' - only the sensor's own bridge offset is removed by SET/RESET.
  bool mag_offset_valid = false;
  float mag_offset_uT[3] = {0.0f, 0.0f, 0.0f};
  float mag_setreset_field_uT[3] = {0.0f, 0.0f, 0.0f};
  float mag_offset_std_uT = 0.0f;   // largest per-axis std of the single offset values
  uint8_t mag_offset_n = 0;         // number of SET/RESET pairs averaged
};

// Configures IMU + sensor-hub magnetometer. Returns false (and skips the
// configuration) if the IMU does not answer with the expected WHO_AM_I.
bool setup_LSM6DSV16X(ImuSite& site);

// One 12-byte burst read of gyro + accel output registers. Returns false on
// any I2C error (outputs are then left unchanged).
bool read_imu(ImuSite& site, float* accel_mg, float* angrate_mdps);

// Reads the MMC5603 data cached in the IMU's sensor-hub registers (one 9-byte
// burst). Returns false on any I2C error or if the sensor hub reported a NACK
// from the magnetometer (outputs are then left unchanged). With
// MAG_SUBTRACT_SETRESET_OFFSET (sensor_cfg.h) the boot-time SET/RESET offset
// is subtracted (only if that measurement succeeded).
bool read_magnetometer(ImuSite& site, double* mag_field_uT);

#endif
