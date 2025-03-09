/*!
 * @file rt_udprc.h
 * @brief Udp-rc communication to motor board
 */

#ifndef _rt_udprc
#define _rt_udprc

#include <fcntl.h>      //Needed for UDP-rc port
#include <sys/ioctl.h>  //Needed for UDP-rc port

// incredibly obscure bug in UDP-rc_IOC_MESSAGE macro is fixed by this
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
#include <unistd.h>  //Needed for UDP-rc port
// #include <rc_status_t.hpp>
// #include <rc_data_t.hpp>

#include "rt/UdpRCServer.h"

// #define RC_BYTES_PER_MESSAGE 32

#define RC_EXPECTED_DATA_SIZE 52
#define RC_EXPECTED_STATUS_SIZE 30

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                                                                             \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), (byte & 0x08 ? '1' : '0'), \
      (byte & 0x04 ? '1' : '0'), (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

void init_udprc();

void udprc_send_receive(rc_data_t *rc_data_rx, rc_status_t *rc_status_tx);
void udprc_driver_run();

rc_to_lcm_T *get_udprc_data();
// rc_status_t* get_udprc_status();

#endif
