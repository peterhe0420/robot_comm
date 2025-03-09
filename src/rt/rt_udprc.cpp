/*!
 * @file rt_udprc.cpp
 * @author spinor zhu
 * @date 2023-09-08
 * @brief Udp-rc communication
 */

#include "rt/rt_udprc.h"

#include <byteswap.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include "common/include/printLog.h"
#include "rt/UdpRCServer.h"

// #include <rc_status_t.hpp>
// #include <rc_data_t.hpp>
#include <lcm/lcm-cpp.hpp>

pthread_mutex_t udprc_mutex = PTHREAD_MUTEX_INITIALIZER;

extern rc_to_lcm_T rc2lcm_cmd;
rc_data_t rc_data_rx, sula_data_rx;
rc_status_t rc_status_tx, sula_status_tx;

UdpRCServer rcConn;

/*!
 * Initialize UDP-rc
 */
void init_udprc() {
  // check sizes:
  size_t status_size = sizeof(rc_status_tx);
  size_t data_size = sizeof(rc_data_rx);

  memset(&rc_status_tx, 0, sizeof(rc_status_tx));
  memset(&rc_data_rx, 0, sizeof(rc_data_rx));

  if (pthread_mutex_init(&udprc_mutex, NULL) != 0) LOG_INFO("[ERROR: RT UDP-rc] Failed to create udprc data mutex\n");

  if (status_size != RC_EXPECTED_STATUS_SIZE) {
    LOG_INFO("[RT UDP-rc] Error status size is %ld, expected %d\n", status_size, RC_EXPECTED_STATUS_SIZE);
  } else
    LOG_INFO("[RT UDP-rc] status size good\n");

  if (data_size != RC_EXPECTED_DATA_SIZE) {
    LOG_INFO("[RT UDP-rc] Error data size is %ld, expected %d\n", data_size, RC_EXPECTED_DATA_SIZE);
  } else
    LOG_INFO("[RT UDP-rc] data size good\n");

  int ret = rcConn.init(RCPORT, (char *)RCADDR, SuLaPORT, (char *)SuLaADDR);
  if (ret) {
    LOG_INFO("\n[RT UDP-rc] init err\n");
  } else {
    LOG_INFO("\n[RT UDP-rc] init ok\n");
  }
  // rcConn.regCallback(UdpRCClient::recvCallBack, this);

  rcConn.listen();

  //   rcMgmt.regCallback(callme, nullptr);
}

/*!
 * convert udprc command to sula contorl command sula_data_t
 */
void udprc_to_sula(rc_data_t *udprc_data, rc_data_t *sula_data) { memcpy(sula_data, udprc_data, sizeof(rc_data_t)); }

/*!
 * convert sula_tatus to  udprc_status sula_status_t
 */
void sula_to_udprc(rc_status_t *sula_tatus, rc_status_t *udprc_status) { memcpy(udprc_status, sula_tatus, sizeof(rc_status_t)); }

void udprc_send_receive(rc_data_t *rc_data_rx, rc_status_t *rc_status_tx) { usleep(100000); }

/*!
 * Run UDP-rc
 */
void udprc_driver_run() {
  pthread_mutex_lock(&udprc_mutex);
  udprc_send_receive(&rc_data_rx, &rc_status_tx);
  pthread_mutex_unlock(&udprc_mutex);
}

rc_to_lcm_T *get_udprc_data() { return &rc2lcm_cmd; }

// rc_status_t *get_udprc_status() { return &rc_status_tx; }
