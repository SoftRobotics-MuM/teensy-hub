#ifndef sensor_pcb_v2_h
#define sensor_pcb_v2_h

#include <stdint.h>
#include <soro_msgs/msg/sensor_board.h>

// Sets up SPI1 for the shared sensor bus and broadcasts a panic-release
// (every board releases both its IMU CS lines) so the bus starts from a
// known-idle state regardless of what the previous run left it in. Call
// once from setup().
void setup_sensor_pcb_v2();

// Reads both onboard IMUs and both onboard magnetometers of the board at
// targetAddr (0..7) and fills 'board' with physical-unit values (see
// sensor_cfg.h for the conversion constants) and per-sensor validity bits.
// Fields for a sensor that failed to respond/verify are left at whatever
//'board' already held - callers polling into a persistent message buffer
// get "last known good" for free; callers wanting a clean slate should
// zero-initialize 'board' first.
void read_sensor_board(uint8_t targetAddr, soro_msgs__msg__SensorBoard &board);

#endif
