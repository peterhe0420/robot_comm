/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */
// clang-format off
#ifndef _rt_spi
#define _rt_spi



#include <fcntl.h>      //Needed for SPI port
#include <sys/ioctl.h>  //Needed for SPI port

// incredibly obscure bug in SPI_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

#include <linux/spi/spidev.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  //Needed for SPI port
#include "lcm-types/cpp/spi_command_t.hpp"
#include "lcm-types/cpp/spi_data_t.hpp"
#include "lcm-types/cpp/spi_torque_t.hpp"
#include "MotorMgmt.h"

#define K_EXPECTED_COMMAND_SIZE 256
#define K_WORDS_PER_MESSAGE 69
#define K_EXPECTED_DATA_TAU 116
#define K_EXPECTED_DATA_NO_TAU 164
#define K_EXPECTED_DATA_SIZE K_EXPECTED_DATA_NO_TAU //164//132//
#define K_KNEE_OFFSET_POS 2.585 // PI/2 //spinor148.11
#define K_ABAD_OFFSET_POS PI/4 // 

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'),     \
      (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), \
      (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')
// clang-format on

void init_spi();

void spi_send_receive(spi_command_t *command, spi_data_t *data);
void spi_driver_run();
void temperature(spi_command_t *tcommand, spi_data_t *tdata);

spi_data_t *get_spi_data();
spi_command_t *get_spi_command();

/*!
 * SPI command message 1+15*2+2+1 U32=34*2 U16= 68
 */
typedef struct {
  int32_t _pad;
  float q_des_abad[2];
  float q_des_hip[2];
  float q_des_knee[2];

  float qd_des_abad[2];
  float qd_des_hip[2];
  float qd_des_knee[2];

  float kp_abad[2];
  float kp_hip[2];
  float kp_knee[2];

  float kd_abad[2];
  float kd_hip[2];
  float kd_knee[2];

  float tau_abad_ff[2];
  float tau_hip_ff[2];
  float tau_knee_ff[2];

  int32_t flags[2];
  int32_t checksum;

} spine_cmd_t;

/*!
 * SPI data message
 */
typedef struct {
  float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  float tau_abad[2];
  float tau_hip[2];
  float tau_knee[2];
  int32_t flags[2];
  int32_t checksum;

} spine_data_t;

#endif
