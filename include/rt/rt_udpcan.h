/*!
 * @file rt_udpcan.h
 * @brief Udp-Can communication to motor board
 */

#ifndef _rt_udpcan
#define _rt_udpcan

#include <fcntl.h>      //Needed for UDP-CAN port
#include <sys/ioctl.h>  //Needed for UDP-CAN port

// incredibly obscure bug in UDP-CAN_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  //Needed for UDP-CAN port

#include "MotorMgmt.h"
#include "lcm-types/cpp/spi_command_t.hpp"
#include "lcm-types/cpp/spi_data_t.hpp"
#include "lcm-types/cpp/spi_torque_t.hpp"


#define K_EXPECTED_COMMAND_SIZE 256
#define K_WORDS_PER_MESSAGE 66
#define K_EXPECTED_DATA_SIZE 116
#define K_KNEE_OFFSET_POS 4.35f

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                                                                             \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), (byte & 0x08 ? '1' : '0'), \
      (byte & 0x04 ? '1' : '0'), (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

void init_udpcan();

void udpcan_send_receive(spi_command_t *command, spi_data_t *data);
void udpcan_driver_run();

spi_data_t *get_udpcan_data();
spi_command_t *get_udpcan_command();

/*!
 * UDP-CAN command message
 */
typedef struct {
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];
  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];
  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];
  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];
  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];
  int32_t flags[4];
  int32_t checksum;

} motor_cmd_t;

/*!
 * UDP-CAN data message
 */
typedef struct {
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  float tau_abad[4];
  float tau_hip[4];
  float tau_knee[4];
  int32_t flags[4];
  int32_t checksum;

} motor_data_t;

#endif
