#ifndef MICROAUTOWARE_CONFIG_H_
#define MICROAUTOWARE_CONFIG_H_

  // microAutoware node name
  #define NODE_NAME "vehicle_interface"

  // microAutoware transport layer
  #define TRANSPORT UART

  // Use simulation time (0 - false; 1 - true)
  #define USE_SIM_TIME 0

  // Timeout for sync timestamp with ROS
  #define TIMEOUT_TS_SYNC 60

  // Timeout ping to micro-ros agent
  #define WATCHDOG_AGENT_TIMEOUT 1000

  // Executor spin once timeout in ms
  #define EXECUTOR_SPIN_TIME 20

#endif
