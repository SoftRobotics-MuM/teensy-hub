# All sensor-pcb-v2 board data in one message/topic, regardless of how many
# boards are physically present (1 today, up to 8 later) or which hardware
# generation (old I2C or new shared-SPI-bus) is compiled into the firmware -
# this keeps the ROS-side interface identical across both.

uint8 MAX_BOARDS = 8

SensorBoard[8] boards
uint8 boards_populated   # bitmask: bit i set <=> boards[i] holds real data
