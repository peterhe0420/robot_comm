/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: IMU service module
**************************************************/
#ifndef IMU_SERVICE_H_
#define IMU_SERVICE_H_

#include <math.h>
// #include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include "comm.h"
#include "control_msgs/imu_data.h"
#include "serial/serial.h"

// #include <thread>

using namespace std;

#define IMU_YIS300_DK_ACC 0x10
#define IMU_YIS300_DK_ANGULAR_VEL 0x20
#define IMU_YIS300_DK_QUAD 0x41

#define UARTIMU
#ifdef UARTIMU
const string IMU_DEV_NAME = "/dev/ttyTHS0";
#else
const string IMU_DEV_NAME = "/dev/ttyUSB0";
#endif
const uint32_t IMU_SERIAL_BAUDRATE = 460800;
const uint kTatalBufSize = 4096 * 2;

enum IMU_PKG_IDX {
  IMU_PKG_START_IDX = 0,
  IMU_PKG_ANGLE_VEL_X_IDX = 3,
  IMU_PKG_ANGLE_VEL_Y_IDX = 7,
  IMU_PKG_ANGLE_VEL_Z_IDX = 11,
  IMU_PKG_ACC_X_IDX = 15,
  IMU_PKG_ACC_Y_IDX = 19,
  IMU_PKG_ACC_Z_IDX = 23,
  IMU_PKG_FRM_COUNTER = 31,
  IMU_PKG_CRC = 33,
};

enum IMU_TYPE {
  IMU_F5101NAC = 0,
  IMU_YIS300_DK = 1,
};

struct IMU_DATA {
  float vx, vy, vz;
  float ax, ay, az;
  float qw, qx, qy, qz;
};

class IMUService {
 private:
  IMU_DATA imu_data;
  vector<int> mounting_matrix;
  vector<float> mounting_quat_inv;
  serial::Serial *ser = NULL;

  int dataIndex = 0;
  uint8_t totalBuf[kTatalBufSize] = {0};
  int sendTopicIndex = -1;
  int fpsCnt = 0;
  int errCnt = 0;

  timeval recvTime, lastTime;

  // ros::Publisher infoPub;

  inline uint8_t getCRC(const uint8_t *buf, uint8_t len);  // Get checksum value
  inline float uint8toFloat(const uint8_t *buf, int idx);  // uint8转float
  inline int32_t uint8to32(const uint8_t *buf, int idx);   // uint8转float
  inline void computeAngularVel(const uint8_t *buf);
  inline void computeAcc(const uint8_t *buf);

  inline void parseACC(const uint8_t *buf);
  inline void parseAngularVel(const uint8_t *buf);
  inline void parseQUAD(const uint8_t *buf);

  double timeval_diff(struct timeval *tv0, struct timeval *tv1) {
    double time1, time2;
    time1 = tv0->tv_sec * 1000.0 + (tv0->tv_usec / 1000.0);
    time2 = tv1->tv_sec * 1000.0 + (tv1->tv_usec / 1000.0);
    time1 = time1 - time2;
    if (time1 < 0) time1 = -time1;
    return time1;
  }

  void pubMsg();
  bool checkCRC(const uint8_t *buf, IMU_TYPE type);
  bool checkHeader(const uint8_t *buf, IMU_TYPE type);
  void parseData(const uint8_t *buf, IMU_TYPE type);

 public:
  IMUService() {}
  bool init();
  void procOnce(IMU_TYPE type);
  IMU_DATA getData();

  ~IMUService() {
    if (NULL != ser && ser->isOpen()) ser->close();
  }
};

#endif