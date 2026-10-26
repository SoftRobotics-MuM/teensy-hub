# All sensor-pcb-v2 board data in one message/topic, regardless of how many
# boards are physically present (1 today, up to 8 later) or which hardware
# generation (old I2C or new shared-SPI-bus) is compiled into the firmware -
# this keeps the ROS-side interface identical across both.

uint8 MAX_BOARDS = 8

# Time at which the Teensy started reading this sample set (all boards are
# read sequentially right after it, within a few ms), in the micro-ROS
# agent host's clock (rmw_uros_epoch_nanos(), re-synchronised every few
# seconds). Zero (sec == 0 and nanosec == 0) while the Teensy has not yet
# synchronised its clock with the agent - consumers should then fall back
# to their own receive time.
builtin_interfaces/Time stamp

SensorBoard[8] boards
uint8 boards_populated   # bitmask: bit i set <=> boards[i] holds real data
