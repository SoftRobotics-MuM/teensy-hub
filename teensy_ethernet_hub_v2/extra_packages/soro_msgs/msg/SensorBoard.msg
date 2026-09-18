# One sensor-pcb-v2 board's data (see soro_msgs/msg/SensorArray.msg).
#
# On old-hardware (I2C LSM6DSV16X+MMC5603) builds, only imu1/mag1 are
# populated - imu2/mag2 stay zero with their status bits clear, since the
# old hardware has no second onboard IMU/magnetometer per site.
#
# All physical units follow ROS REP-103 (SI): rad/s, m/s^2, Tesla.

uint8 STATUS_IMU1_VALID = 1  # bit 0
uint8 STATUS_IMU2_VALID = 2  # bit 1
uint8 STATUS_MAG1_VALID = 4  # bit 2
uint8 STATUS_MAG2_VALID = 8  # bit 3

uint8 status    # bitmask of the STATUS_* constants above
uint8 seq       # rolling mag-cache sequence counter (new hw only; 0 on old hw)

# gx, gy, gz, ax, ay, az
float32[6] imu1
float32[6] imu2

# x, y, z
float32[3] mag1
float32[3] mag2
