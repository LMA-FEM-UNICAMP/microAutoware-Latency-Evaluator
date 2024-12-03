/**
  ******************************************************************************
  * @file    microAutoware.h
  * @brief   This file contains the declarations for FreeRTOS task microAutoware, 
  *          for implementing a vehicle interface for Autoware using micro-ros.
  ******************************************************************************
  * @author  Gabriel Toffanetto França da Rocha 
  *          Laboratory of Autonomous Vehicles (LMA) - FEM/Unicamp
  * @date    Created:  October 9, 2024
  *          Modified: 
  ******************************************************************************
  */
#ifndef MICROAUTOWARE_H_
#define MICROAUTOWARE_H_

  #define UART 1
  #define UDP 2
  #define USB 3
  #define OTHER 0

  #define NODE_NAME "vehicle_interface"
  #define TRANSPORT UART


  #if TRANSPORT == UART
    #include "usart.h"
    #include "dma.h"
  #endif  /* TRANSPORT == UART */

  // Includes -- START
  #include <rcl/rcl.h>
  #include <rcl/error_handling.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <uxr/client/transport.h>
  #include <rmw_microxrcedds_c/config.h>
  #include <rmw_microros/rmw_microros.h>

  // Autoware msg types
  #include <autoware_control_msgs/msg/control.h>
  #include <autoware_vehicle_msgs/msg/control_mode_report.h>
  #include <autoware_vehicle_msgs/msg/engage.h>
  #include <autoware_vehicle_msgs/msg/gear_command.h>
  #include <autoware_vehicle_msgs/msg/gear_report.h>
  #include <autoware_vehicle_msgs/msg/hazard_lights_command.h>
  #include <autoware_vehicle_msgs/msg/hazard_lights_report.h>
  #include <autoware_vehicle_msgs/msg/steering_report.h>
  #include <autoware_vehicle_msgs/msg/turn_indicators_command.h>
  #include <autoware_vehicle_msgs/msg/turn_indicators_report.h>
  #include <autoware_vehicle_msgs/msg/velocity_report.h>
  #include <autoware_vehicle_msgs/srv/control_mode_command.h>
  #include <tier4_vehicle_msgs/msg/actuation_command_stamped.h>
  #include <tier4_vehicle_msgs/msg/actuation_status_stamped.h>
  #include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.h>
  #include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.h>
  // Includes -- END


  // Function Prototypes -- START

  // Task prototype
  void StartMicroAutoware(void *argument);

  // MicroROS prototypes
  bool cubemx_transport_open(struct uxrCustomTransport * transport);
  bool cubemx_transport_close(struct uxrCustomTransport * transport);
  size_t cubemx_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err);
  size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, int timeout, uint8_t* err);

  void * microros_allocate(size_t size, void * state);
  void microros_deallocate(void * pointer, void * state);
  void * microros_reallocate(void * pointer, size_t size, void * state);
  void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

  // Executor callbacks (implemented in executorCallbacks.c)
  void control_cmd_callback(const void * xMsgIn);
  void gear_cmd_callback(const void * xMsgIn);
  void turn_indicators_cmd_callback(const void * xMsgIn);
  void hazard_lights_cmd_callback(const void * xMsgIn);
  void actuation_cmd_callback(const void * xMsgIn);
  void emergency_callback(const void * xMsgIn);
  void control_mode_cmd_callback(const void * xRequestMsg, void * xResponseMsg);

  // Function Prototypes -- END

#endif /* MICROAUTOWARE_H_ */
