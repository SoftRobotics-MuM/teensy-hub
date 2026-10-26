// https://docs.vulcanexus.org/en/latest/rst/microros_documentation/benchmarking/benchmarking.html?utm_source=chatgpt.com
// pio pkg install # Install dependencies
// pio run # Build the firmware
// pio run --target upload # Flash the firmware
// pio run --target clean_microros  # Clean library

/* How to run
* first start docker container with micro-ROS agent:
* docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
* then run this example on Arduino
* Ethernet addresses/ports:
* IP arduino: 192.168.1.177
* IP agent: 192.168.1.113
* Port: 8888
* 
* ros2 topic pub /teensy_hub/subscribe_topic std_msgs/msg/Int32 "{data: 10}"
* ros2 topic pub /teensy_hub/subscribe_topic soro_msgs/msg/ServoCommands "{servo_micros: [1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510, 1511, 1512]}"
*
* ros2 topic echo /teensy_hub/servo_pos_set
* ros2 topic pub /teensy_hub/servo_pos soro_msgs/msg/ServoCommands "{servo_micros: [1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510, 1511, 1512]}"
* ros2 topic pub /teensy_hub/servo_pos soro_msgs/msg/ServoCommands "{servo_micros: [15100, 15200, 15300, 15400, 15500, 15600, 15700, 15800, 15900, 15100, 15110, 15120]}"
*
* ros2 run plotjuggler plotjuggler
*/

/*
* Fix ERROR:colcon.colcon_cmake.task.cmake.build:Could not find a shell extension for the command environment Failed <<< ament_cmake_python [0.02s, exited with code 1]
* clean everything:
* pio run --target clean
* rm -rf .pio/libdeps/
* rm -rf ~/.micro_ros_dev
*
* update extensions
* sudo apt update
* sudo apt install python3-colcon-common-extensions python3-pip
* pip3 install -U setuptools
*
*Force-rebuild micro-ROS dev deps
* cd .pio/libdeps/teensy41_eth/micro_ros_platformio
* rm -rf build/ install/ log/
* pio run
*
* If that still fails with "No 'rosidl_typesupport_cpp' found" (CMake error
* inside builtin_interfaces/rosidl_typesupport_c): a system ROS 2 install
* sourced in your shell profile (e.g. `source /opt/ros/jazzy/setup.zsh` in
* .zshrc) leaks CMAKE_PREFIX_PATH/AMENT_PREFIX_PATH into this build, and it
* picks up the host ROS's C++ typesupport instead of the bundled C-only one
* this cross-compiled target needs. Rebuild with those stripped, e.g.:
*   env -u CMAKE_PREFIX_PATH -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u COLCON_PREFIX_PATH \
*     PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '/opt/ros/jazzy' | paste -sd: -)" \
*     pio run
*/

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <rmw_microros/rmw_microros.h> // rmw_uros_sync_session(), rmw_uros_epoch_nanos(), ping
#include <soro_msgs/msg/servo_commands.h>
#include "servo_control.h"
#include "servo_cfg.h"
#include "sensor_cfg.h"
#include <soro_msgs/msg/sensor_array.h>
#if USE_NEW_SENSOR_PCB
  #include "sensor_pcb_v2.h"
#else
  #include <read_LSM6DSV16X.h>
  #include "LSM6DSV16X_regs.h"
  #include "MMC5603_regs.h"
#endif


#if !defined(ARDUINO_TEENSY41)
  #error This code only runs on Arduino Teensy41
#endif

// Used while creating the micro-ROS entities: on any failure, give up this
// attempt (the connection state machine in loop() tears down and retries).
#define RCRETURN_FALSE(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// parameters
// Note: The publish period times can be set to a value higher than 1000*1000*60 to effectively disable the respective publisher.
const uint32_t publish_period_time_servo_set_micros = 20*1000;//1000*1000*60; // period time in microseconds
const uint32_t publish_period_time_sensor_data_micros = 10*1000; // period time in microseconds (100 Hz)

// Clock synchronisation with the micro-ROS agent (used for SensorArray.stamp).
// Done ONCE, at the first micro-ROS session after power-up, never during
// operation and not after a reconnection (the offset refers to our own local
// clock, so it stays valid across sessions): a runtime sync blocks the loop
// and needs a symmetric, short round trip, which is not guaranteed under
// load. Recordings are short enough for the Teensy crystal's drift. If the
// sync fails, it is retried at the next session start.
//
// The measured offset is stored here and applied to our own 64-bit local
// clock (local_nanos()), instead of using rmw_uros_epoch_nanos() for every
// stamp. This makes the stamps immune to (a) sync replies that arrive late,
// after a timed-out attempt, and would otherwise silently overwrite the
// session's offset later with a wrong value, and (b) the 32-bit rollover bug
// of micro_ros_platformio's clock_gettime() (jump after ~71.6 min uptime).
// A sync attempt is accepted only if it returned within TIME_SYNC_MAX_RTT_MS
// and two consecutive accepted attempts agree within TIME_SYNC_AGREE_NS.
const int      TIME_SYNC_TIMEOUT_MS  = 50;       // per attempt
const int      TIME_SYNC_ATTEMPTS    = 20;       // max attempts per session start
const uint32_t TIME_SYNC_MAX_RTT_MS  = 10;       // reject slower round trips
const int64_t  TIME_SYNC_AGREE_NS    = 2000000;  // 2 ms

// micro-ROS agent connection handling. The firmware no longer halts when the
// agent is missing or lost: it waits for the agent, (re)creates all micro-ROS
// entities when it appears, and tears them down when it disappears.
// While connected, receiving servo commands proves the connection works, so
// the liveness ping only runs once no servo command (valid or not) has been
// received for AGENT_PING_IDLE_MS - never during normal operation.
const uint32_t AGENT_WAIT_PING_PERIOD_MS  = 500;  // ping interval while waiting for the agent
const int      AGENT_WAIT_PING_TIMEOUT_MS = 100;
const uint32_t AGENT_PING_IDLE_MS         = 30000; // no servo command for this long -> start pinging
const uint32_t AGENT_PING_PERIOD_MS       = 1000; // liveness ping interval while idle
const int      AGENT_PING_TIMEOUT_MS      = 50;   // generous: pings never run during experiments
const uint8_t  AGENT_PING_MAX_FAILURES    = 5;    // consecutive failed pings (~5 s) => agent lost

// Servo failsafe: if the micro-ROS agent has been unreachable for this long,
// all servos are moved to CENTER_SERVO_US. While the agent is connected the
// servos hold their last command indefinitely (e.g. static tests), however
// rarely new commands arrive.
const uint32_t AGENT_LOST_SERVO_TIMEOUT_MS = 60000;

// Note: Pin 13 (Teensy onboard LED) is also PIN_MOTOR_12's PWM output (see servo_cfg.h).
// Do not drive it with digitalWrite() - that would fight the hardware PWM and glitch motor 12.
// The LED simply reflects motor 12's PWM duty cycle as a side effect.

// micro-ROS configuration
const size_t ROS_DOMAIN_ID = 72;
rcl_publisher_t publisher;
rcl_publisher_t publisher_sensor_data;

rcl_subscription_t subscriber;
soro_msgs__msg__ServoCommands current_servo_pos_msg;
soro_msgs__msg__ServoCommands incoming_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// QoS profile for the servo command channel (best-effort, depth 1): always act on the
// newest command rather than draining a backlog if the PC ever publishes faster than
// the Teensy consumes.
rmw_qos_profile_t qos_profile_servo_cmd = rmw_qos_profile_sensor_data;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

// Clock offset to the agent host (see TIME_SYNC_*)
bool    time_synced = false;
int64_t time_offset_ns = 0;   // agent time - local_nanos()

// Last time any servo command message arrived (see AGENT_PING_IDLE_MS)
uint32_t last_servo_msg_ms = 0;
uint8_t  agent_ping_failures = 0;

// Servo failsafe + diagnostics
uint32_t agent_lost_since_ms = 0;  // start of the current period without agent connection
bool     servos_in_failsafe = false;
uint32_t rejected_servo_commands = 0;

// IMU (old hardware only - see sensor_cfg.h's USE_NEW_SENSOR_PCB)
#if !USE_NEW_SENSOR_PCB
const int NUM_IMU = 4; // Number of IMUs (old-hardware sites; maps 1:1 onto SensorArray.boards[0..3])
LSM6DSV16XSensor lsm6dsv16x_1(&Wire1, LSM6DSV16X_I2C_ADD_L);
LSM6DSV16XSensor lsm6dsv16x_2(&Wire1, LSM6DSV16X_I2C_ADD_H);
LSM6DSV16XSensor lsm6dsv16x_3(&Wire2, LSM6DSV16X_I2C_ADD_L);
LSM6DSV16XSensor lsm6dsv16x_4(&Wire2, LSM6DSV16X_I2C_ADD_H);

// The sites refer to the sensor objects above (no copies - the library stores
// 'this' in its register context, so a copy would silently keep using the
// original object's state). Bus + 7-bit address are repeated here for the
// error-checked direct I2C reads in read_LSM6DSV16X.cpp (the library's own
// I2C helpers ignore bus errors and always report success).
ImuSite imus[NUM_IMU] = {
  {lsm6dsv16x_1, Wire1, (uint8_t)(LSM6DSV16X_I2C_ADD_L >> 1)},
  {lsm6dsv16x_2, Wire1, (uint8_t)(LSM6DSV16X_I2C_ADD_H >> 1)},
  {lsm6dsv16x_3, Wire2, (uint8_t)(LSM6DSV16X_I2C_ADD_L >> 1)},
  {lsm6dsv16x_4, Wire2, (uint8_t)(LSM6DSV16X_I2C_ADD_H >> 1)},
};
#endif

#if !USE_NEW_SENSOR_PCB
// Boot-time SET/RESET result per site (see read_LSM6DSV16X.cpp).
void print_mag_offsets() {
  for (int i = 0; i < NUM_IMU; i++) {
    const ImuSite &s = imus[i];
    if (!s.present) {
      continue;
    }
    if (!s.mag_offset_valid) {
      Serial.printf("MMC5603 site %d: SET/RESET offset measurement FAILED\n", i);
      continue;
    }
    const float *o = s.mag_offset_uT, *h = s.mag_setreset_field_uT;
    Serial.printf("MMC5603 site %d: offset (%.2f, %.2f, %.2f) uT, |offset| %.2f uT, std %.2f uT (%u pairs); "
                  "field w/o offset (%.2f, %.2f, %.2f) uT, |field| %.2f uT; offset %s\n",
                  i, o[0], o[1], o[2], sqrtf(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]),
                  s.mag_offset_std_uT, s.mag_offset_n,
                  h[0], h[1], h[2], sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]),
                  MAG_SUBTRACT_SETRESET_OFFSET ? "SUBTRACTED" : "not subtracted");
  }
}
#endif

#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

/**
 * @brief Fixed-rate scheduler: returns true once per period_micros, on an exact
 * grid (next += period) rather than "period after the last call", so loop
 * latency does not accumulate and the long-term rate is exactly 1/period
 * (relative to the Teensy's crystal). If the loop falls behind by more than a
 * whole period (e.g. a blocking call), the grid is restarted from now instead of
 * firing several times back to back. micros() wrap-around is handled by the
 * signed difference.
 */
bool periodic_due(uint32_t &next_micros, bool &started, uint32_t period_micros) {
  const uint32_t now = micros();
  if (!started) {
    started = true;
    next_micros = now + period_micros;
    return true;
  }
  if ((int32_t)(now - next_micros) < 0) {
    return false;
  }
  next_micros += period_micros;
  if ((int32_t)(now - next_micros) >= 0) {
    next_micros = now + period_micros; // more than one period late: resync grid
  }
  return true;
}

/**
 * @brief Local monotonic clock [ns]: micros() extended to 64 bits. Called at
 * least every sample period, far more often than the 71.6 min micros() wrap.
 */
int64_t local_nanos() {
  static uint32_t last_us = 0;
  static uint64_t high = 0;
  const uint32_t now_us = micros();
  if (now_us < last_us) {
    high += (1ULL << 32);
  }
  last_us = now_us;
  return (int64_t)(high + now_us) * 1000LL;
}

/**
 * @brief One-off clock sync with the agent (see TIME_SYNC_*). On success
 * time_offset_ns maps local_nanos() to the agent host's clock; on failure
 * time_synced stays false and the stamps stay zero.
 */
bool sync_time_with_agent() {
  time_synced = false;
  bool have_prev = false;
  int64_t prev_offset = 0;
  for (int i = 0; i < TIME_SYNC_ATTEMPTS; i++) {
    const uint32_t t0 = millis();
    if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) != RMW_RET_OK) {
      continue;
    }
    if (millis() - t0 > TIME_SYNC_MAX_RTT_MS) {
      continue;  // slow round trip -> asymmetric delay likely, offset unreliable
    }
    const int64_t offset = rmw_uros_epoch_nanos() - local_nanos();
    if (have_prev && llabs(offset - prev_offset) <= TIME_SYNC_AGREE_NS) {
      time_offset_ns = offset;
      time_synced = true;
      return true;
    }
    prev_offset = offset;
    have_prev = true;
  }
  return false;
}

/**
 * @brief Move all servos to CENTER_SERVO_US.
 */
void center_all_servos() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].set_pulse_length(CENTER_SERVO_US);
  }
}

/**
 * @brief Servo failsafe: center all servos once if the agent has been
 * unreachable for AGENT_LOST_SERVO_TIMEOUT_MS. Runs every loop iteration. After
 * reconnection, the next valid command takes over again.
 */
void servo_failsafe_check() {
  if (agent_state == AGENT_CONNECTED) {
    agent_lost_since_ms = millis();
    return;
  }
  if (!servos_in_failsafe && (millis() - agent_lost_since_ms >= AGENT_LOST_SERVO_TIMEOUT_MS)) {
    center_all_servos();
    servos_in_failsafe = true;
    Serial.println("Servo failsafe: agent unreachable for AGENT_LOST_SERVO_TIMEOUT_MS - servos centered");
  }
}

/**
 * @brief Publish the servo positions currently set in the servo library to the "servo_pos_set" topic.
 * Note: This is the position sent to the servos via PWM, not neccessarily the actual position of the servos.
 * 
 * @param publish_interval_ms The interval in milliseconds at which the servo positions are published.
 * 
 * This function is called in the main loop to periodically publish the current servo positions.
 * It retrieves the current servo positions from the ServoControl objects and publishes them to the "servo_pos_set" topic.
 * The function uses a static variable to keep track of the last time it was called, ensuring that it only publishes at the specified interval.
 */
void publish_current_servo_positions(uint32_t publish_interval_micros = 1000000) {
  static uint32_t next_call_micros = 0;
  static bool started = false;
  if (!periodic_due(next_call_micros, started, publish_interval_micros)) {
    return; // fixed-rate: publish once per publish_interval_micros
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    current_servo_pos_msg.servo_micros[i] = motors[i].get_servo_pulse_length_set(); // Get the pulse length set for each servo
  }
  RCSOFTCHECK(rcl_publish(&publisher, &current_servo_pos_msg, NULL));
}


/**
 * @brief Read all sensor data (old I2C hardware or new sensor-pcb-v2 shared-SPI-bus
 * hardware, per sensor_cfg.h's USE_NEW_SENSOR_PCB) and publish it as one
 * soro_msgs::msg::SensorArray on "sensor_data" - see SensorArray.msg/SensorBoard.msg
 * for the wire layout. Rate-limited like the other publish_* functions.
 */
soro_msgs__msg__SensorArray sensor_data_msg;
bool publish_sensor_data(uint32_t publish_interval_micros = 10000) {
  static uint32_t next_call_micros = 0;
  static bool started = false;
  if (!periodic_due(next_call_micros, started, publish_interval_micros)) {
    return false; // fixed-rate: publish once per publish_interval_micros
  }

  // Timestamp = start of this read cycle, in the agent host's clock. Zero
  // if the clock sync at session start failed (see SensorArray.msg).
  if (time_synced) {
    const int64_t t_ns = local_nanos() + time_offset_ns;
    sensor_data_msg.stamp.sec = (int32_t)(t_ns / 1000000000LL);
    sensor_data_msg.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);
  } else {
    sensor_data_msg.stamp.sec = 0;
    sensor_data_msg.stamp.nanosec = 0;
  }

  sensor_data_msg.boards_populated = 0;

#if USE_NEW_SENSOR_PCB
  for (uint8_t addr = 0; addr < MAX_SENSOR_BOARDS; addr++) {
    if (!BOARD_ENABLED[addr]) {
      continue;
    }
    read_sensor_board(addr, sensor_data_msg.boards[addr]);
    sensor_data_msg.boards_populated |= (1 << addr);
  }
#else
  for (int i = 0; i < NUM_IMU; i++) {
    soro_msgs__msg__SensorBoard &board = sensor_data_msg.boards[i];
    board.status = 0;
    board.seq = 0;

    // Sites whose IMU did not answer at startup are not populated at all.
    if (!imus[i].present) {
      continue;
    }
    sensor_data_msg.boards_populated |= (1 << i);

    // Valid bits are only set if the I2C transfers actually succeeded (and,
    // for the magnetometer, the IMU's sensor hub reported no NACK from it).
    // Fields of an invalid sensor keep their previous values - consumers must
    // check the status bits.
    float accel_mg[3];
    float angrate_mdps[3];
    if (read_imu(imus[i], accel_mg, angrate_mdps)) {
      board.imu1[0] = angrate_mdps[0] * MDPS_TO_RADPS;
      board.imu1[1] = angrate_mdps[1] * MDPS_TO_RADPS;
      board.imu1[2] = angrate_mdps[2] * MDPS_TO_RADPS;
      board.imu1[3] = accel_mg[0] * MG_TO_MPS2;
      board.imu1[4] = accel_mg[1] * MG_TO_MPS2;
      board.imu1[5] = accel_mg[2] * MG_TO_MPS2;
      board.status |= soro_msgs__msg__SensorBoard__STATUS_IMU1_VALID;
    }

    double mag_field_uT[3];
    if (read_magnetometer(imus[i], mag_field_uT)) {
      board.mag1[0] = mag_field_uT[0] * 1e-6f;
      board.mag1[1] = mag_field_uT[1] * 1e-6f;
      board.mag1[2] = mag_field_uT[2] * 1e-6f;
      board.status |= soro_msgs__msg__SensorBoard__STATUS_MAG1_VALID;
    }

#if MAG_PUBLISH_SETRESET_OFFSET
    // Boot-time SET/RESET offset of this site's MMC5603 [T] in the otherwise
    // unused mag2 (STATUS_MAG2_VALID stays clear; zero if not measured).
    for (int a = 0; a < 3; a++) {
      board.mag2[a] = imus[i].mag_offset_valid ? imus[i].mag_offset_uT[a] * 1e-6f : 0.0f;
    }
#endif
  }
#endif

  RCSOFTCHECK(rcl_publish(&publisher_sensor_data, &sensor_data_msg, NULL));
  return true;
}

// Subscriber callback function
// A command is applied only if ALL its values are valid pulse lengths (see
// ServoControl::is_valid_pulse_length(): within [MIN_SERVO_US, MAX_SERVO_US],
// given in us or in 0.1 us). Otherwise the whole message is rejected and the
// servos keep their previous setpoints - never a partially applied command.
void servo_command_subscription_callback(const void * msgin)
{
  const soro_msgs__msg__ServoCommands * msg_in = (const soro_msgs__msg__ServoCommands *)msgin;

  // Any received command (valid or not) proves the agent connection works.
  last_servo_msg_ms = millis();
  agent_ping_failures = 0;

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].is_valid_pulse_length(msg_in->servo_micros[i])) {
      rejected_servo_commands++;
      // Rate-limited so a stream of bad commands cannot flood the serial port.
      static uint32_t last_report_ms = 0;
      if (millis() - last_report_ms >= 1000) {
        last_report_ms = millis();
        Serial.printf("Servo command rejected (servo %d = %u out of range), %lu rejected so far\n",
                      i + 1, (unsigned)msg_in->servo_micros[i], (unsigned long)rejected_servo_commands);
      }
      return;
    }
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].set_pulse_length(msg_in->servo_micros[i]); // Set servo pulse length based on received message
  }
  servos_in_failsafe = false;
}

/**
 * @brief Create all micro-ROS entities (support, node, publishers, subscriber,
 * executor) and do the initial clock sync. Returns false on any failure; the
 * caller then destroys whatever was created and retries later.
 */
bool create_entities() {
  allocator = rcl_get_default_allocator();

  // init_options carry the ROS domain id; rcl_init copies them into the
  // context, so they can be finalized right after the support init.
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCRETURN_FALSE(rcl_init_options_init(&init_options, allocator));
  rcl_ret_t rc = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
  if (rc == RCL_RET_OK) {
    rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  }
  (void)rcl_init_options_fini(&init_options);
  if (rc != RCL_RET_OK) {
    return false;
  }

  // create node
  RCRETURN_FALSE(rclc_node_init_default(&node, "micro_ros_arduino_ethernet_node", "teensy_hub", &support));

  qos_profile_servo_cmd.depth = 1; // keep only the latest servo command/position, never a backlog

  // publisher for current servo positions
  RCRETURN_FALSE(rclc_publisher_init(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
    "servo_pos_set",
    &qos_profile_servo_cmd));

  // publisher for all sensor data (see SensorArray.msg/SensorBoard.msg)
  RCRETURN_FALSE(rclc_publisher_init_best_effort(
    &publisher_sensor_data,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, SensorArray),
    "sensor_data"));

  // subscriber for servo commands
  RCRETURN_FALSE(rclc_subscription_init(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
    "servo_pos",
    &qos_profile_servo_cmd));

  // executor with 1 handle (subscriber)
  executor = rclc_executor_get_zero_initialized_executor();
  RCRETURN_FALSE(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCRETURN_FALSE(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &incoming_msg,
    &servo_command_subscription_callback,
    ON_NEW_DATA));

  // One-off clock sync with the agent (for SensorArray.stamp): only if not
  // yet synchronised since power-up, never re-synced. Not fatal if it fails:
  // the stamps are then zero (the bridge falls back to its receive time) and
  // the sync is retried at the next session start.
  // Fresh session: no liveness ping before AGENT_PING_IDLE_MS without commands.
  last_servo_msg_ms = millis();
  agent_ping_failures = 0;

  if (!time_synced) {
    if (sync_time_with_agent()) {
      Serial.printf("Time synchronised with agent (offset %lld ms)\n", (long long)(time_offset_ns / 1000000LL));
    } else {
      Serial.println("Time sync with agent failed - SensorArray stamps stay 0 until the next session start");
    }
  }
  return true;
}

/**
 * @brief Destroy all micro-ROS entities. Safe to call after a partially failed
 * create_entities() (finalizing a zero-initialized entity just returns an
 * error, which is ignored). The session destroy timeout is set to 0 so that
 * tearing down after the agent is gone does not block waiting for replies.
 */
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  if (rmw_context != NULL) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }

  (void)rcl_publisher_fini(&publisher, &node);
  (void)rcl_publisher_fini(&publisher_sensor_data, &node);
  (void)rcl_subscription_fini(&subscriber, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);

  publisher = rcl_get_zero_initialized_publisher();
  publisher_sensor_data = rcl_get_zero_initialized_publisher();
  subscriber = rcl_get_zero_initialized_subscription();
  node = rcl_get_zero_initialized_node();
}

/**
 * @brief Everything that runs while connected to the agent: publishing, agent
 * liveness ping and the executor. Returns false once the agent
 * is considered lost.
 */
bool run_connected() {
  if (publish_period_time_servo_set_micros < 1000*1000*60) {
    publish_current_servo_positions(publish_period_time_servo_set_micros);
  }

  bool sensor_published = false;
  if (publish_period_time_sensor_data_micros < 1000*1000*60) {
    sensor_published = publish_sensor_data(publish_period_time_sensor_data_micros);
  }

  // Liveness ping, only while no servo commands arrive (see
  // AGENT_PING_IDLE_MS). It blocks briefly, so it is placed right after a
  // sensor publish (idle part of the sample period); if sensor publishing is
  // disabled, it runs whenever due.
  const bool idle_slot = sensor_published || publish_period_time_sensor_data_micros >= 1000*1000*60;
  static uint32_t last_ping_ms = 0;
  if (idle_slot
      && millis() - last_servo_msg_ms >= AGENT_PING_IDLE_MS
      && millis() - last_ping_ms >= AGENT_PING_PERIOD_MS) {
    last_ping_ms = millis();
    if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
      agent_ping_failures = 0;
    } else if (++agent_ping_failures >= AGENT_PING_MAX_FAILURES) {
      agent_ping_failures = 0;
      return false;
    }
  }

  // Spin executor to process incoming messages
  rclc_executor_spin_some(&executor, RCL_US_TO_NS(200));
  return true;
}

void setup() {
  // setup ethernet connection
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };

  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  IPAddress arduino_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 113);
  set_microros_native_ethernet_transports(arduino_mac, arduino_ip, agent_ip, 8888);

  Serial.begin(115200); // Initialize serial output for debug

  // Servo init (the ServoControl constructors already start at center)
  center_all_servos();
  agent_lost_since_ms = millis();

  // Sensor init
#if USE_NEW_SENSOR_PCB
  setup_sensor_pcb_v2();
#else
  Wire1.begin();
  Wire1.setClock(IMU_I2C_CLOCK_HZ);

  Wire2.begin();
  Wire2.setClock(IMU_I2C_CLOCK_HZ);

  for (int i = 0; i < NUM_IMU; i++) {
    imus[i].present = setup_LSM6DSV16X(imus[i]);
    Serial.printf("IMU site %d: %s\n", i, imus[i].present ? "OK" : "NOT RESPONDING (check wiring/address)");
    delay(100);
  }
  print_mag_offsets();
#endif

  // micro-ROS entities are created in loop() as soon as the agent is reachable.
}

void loop() {
  (void)local_nanos();  // keep the 64-bit local clock's wrap detection fed

  // Center servos if the agent stays unreachable (see AGENT_LOST_SERVO_TIMEOUT_MS).
  servo_failsafe_check();

  static uint32_t last_wait_ping_ms = 0;
  switch (agent_state) {
    case WAITING_AGENT:
      if (millis() - last_wait_ping_ms >= AGENT_WAIT_PING_PERIOD_MS) {
        last_wait_ping_ms = millis();
        if (rmw_uros_ping_agent(AGENT_WAIT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
          agent_state = AGENT_AVAILABLE;
        }
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        Serial.println("micro-ROS agent connected");
#if !USE_NEW_SENSOR_PCB
        print_mag_offsets();   // repeated here: the boot output is usually missed
#endif
        agent_state = AGENT_CONNECTED;
      } else {
        Serial.println("micro-ROS entity creation failed - retrying");
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (!run_connected()) {
        Serial.println("micro-ROS agent lost - waiting for reconnection");
        agent_state = AGENT_DISCONNECTED;
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }
}
