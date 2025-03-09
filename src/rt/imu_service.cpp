#include "rt/imu_service.h"

// Get checksum value
inline uint8_t IMUService::getCRC(const uint8_t *buf, uint8_t len) {
  uint8_t ans = 0;
  for (int i = 0; i < len; i++) ans ^= buf[i];
  return ans;
}

bool IMUService::checkCRC(const uint8_t *buf, IMU_TYPE type) {
  if (IMU_F5101NAC == type) {
    uint8_t crc = getCRC(buf, 33);
    return crc == buf[33];
  } else if (IMU_YIS300_DK == type) {
    uint8_t CK1 = 0, CK2 = 0;
    uint len = buf[4];
    for (int i = 2; i < len + 5; i++) {
      CK1 = CK1 + buf[i];
      CK2 = CK2 + CK1;
    }
    return CK1 == buf[len + 5] && CK2 == buf[len + 6];
  } else
    return false;
}

bool IMUService::checkHeader(const uint8_t *buf, IMU_TYPE type) {
  if (IMU_F5101NAC == type) {
    return static_cast<uint8_t>(buf[0]) == 0xBD && static_cast<uint8_t>(buf[1]) == 0xDB && static_cast<uint8_t>(buf[2]) == 0x0A;
  } else if (IMU_YIS300_DK == type) {
    return static_cast<uint8_t>(buf[0]) == 0x59 && static_cast<uint8_t>(buf[1]) == 0x53;
  } else
    return false;
}

void IMUService::parseData(const uint8_t *buf, IMU_TYPE type) {
  if (IMU_F5101NAC == type) {
    computeAcc(buf);         //加速度数据处理函数
    computeAngularVel(buf);  //角速度数据处理函数
  } else if (IMU_YIS300_DK == type) {
    uint len = buf[4], idx = 0;
    const uint8_t *p = buf + 5;
    while (idx < len) {
      uint type = p[idx];
      uint datalen = p[idx + 1];
      switch (type) {
        case IMU_YIS300_DK_ACC:
          parseACC(p + idx + 2);
          break;
        case IMU_YIS300_DK_ANGULAR_VEL:
          parseAngularVel(p + idx + 2);
          break;
        case IMU_YIS300_DK_QUAD:
          parseQUAD(p + idx + 2);
          break;
        default:
          break;
      }
      idx += 2 + datalen;
    }
  }
}

// uint8转float
inline float IMUService::uint8toFloat(const uint8_t *buf, int idx) {
  union change {
    float f_data;
    unsigned char dat[4];
  } i_data;
  for (int i = 0; i < 4; i++) i_data.dat[i] = buf[idx + i];
  return i_data.f_data;
}

// uint8转float
inline int32_t IMUService::uint8to32(const uint8_t *buf, int idx) {
  union change {
    int32_t f_data;
    uint8_t dat[4];
  } i_data;
  for (int i = 0; i < 4; i++) i_data.dat[i] = buf[idx + i];
  return i_data.f_data;
}

inline void IMUService::computeAngularVel(const uint8_t *buf) {
  float x = uint8toFloat(buf, IMU_PKG_ANGLE_VEL_X_IDX);
  float y = uint8toFloat(buf, IMU_PKG_ANGLE_VEL_Y_IDX);
  float z = uint8toFloat(buf, IMU_PKG_ANGLE_VEL_Z_IDX);
  imu_data.vx = mounting_matrix[0] * x + mounting_matrix[1] * y + mounting_matrix[2] * z;
  imu_data.vy = mounting_matrix[3] * x + mounting_matrix[4] * y + mounting_matrix[5] * z;
  imu_data.vz = mounting_matrix[6] * x + mounting_matrix[7] * y + mounting_matrix[8] * z;
  imu_data.vx /= 57.3;
  imu_data.vy /= 57.3;
  imu_data.vz /= 57.3;
}

inline void IMUService::computeAcc(const uint8_t *buf) {
  float x = uint8toFloat(buf, IMU_PKG_ACC_X_IDX);
  float y = uint8toFloat(buf, IMU_PKG_ACC_Y_IDX);
  float z = uint8toFloat(buf, IMU_PKG_ACC_Z_IDX);
  imu_data.ax = mounting_matrix[0] * x + mounting_matrix[1] * y + mounting_matrix[2] * z;
  imu_data.ay = mounting_matrix[3] * x + mounting_matrix[4] * y + mounting_matrix[5] * z;
  imu_data.az = mounting_matrix[6] * x + mounting_matrix[7] * y + mounting_matrix[8] * z;
}

inline void IMUService::parseACC(const uint8_t *buf) {
  float x = uint8to32(buf, 0) / 1.0e6;
  float y = uint8to32(buf, 4) / 1.0e6;
  float z = uint8to32(buf, 8) / 1.0e6;
  imu_data.ax = mounting_matrix[0] * x + mounting_matrix[1] * y + mounting_matrix[2] * z;
  imu_data.ay = mounting_matrix[3] * x + mounting_matrix[4] * y + mounting_matrix[5] * z;
  imu_data.az = mounting_matrix[6] * x + mounting_matrix[7] * y + mounting_matrix[8] * z;
}

inline void IMUService::parseAngularVel(const uint8_t *buf) {
  float x = uint8to32(buf, 0) / 1.0e6;
  float y = uint8to32(buf, 4) / 1.0e6;
  float z = uint8to32(buf, 8) / 1.0e6;
  imu_data.vx = mounting_matrix[0] * x + mounting_matrix[1] * y + mounting_matrix[2] * z;
  imu_data.vy = mounting_matrix[3] * x + mounting_matrix[4] * y + mounting_matrix[5] * z;
  imu_data.vz = mounting_matrix[6] * x + mounting_matrix[7] * y + mounting_matrix[8] * z;
  imu_data.vx /= 57.3;
  imu_data.vy /= 57.3;
  imu_data.vz /= 57.3;
}

inline void IMUService::parseQUAD(const uint8_t *buf) {
  float w = uint8to32(buf, 0) / 1.0e6;
  float x = uint8to32(buf, 4) / 1.0e6;
  float y = uint8to32(buf, 8) / 1.0e6;
  float z = uint8to32(buf, 12) / 1.0e6;
  //  imu_data.qx = x;
  // imu_data.qy = y;
  // imu_data.qz = z;
  // imu_data.qw = w;
  float a = mounting_quat_inv[0];
  float b = mounting_quat_inv[1];
  float c = mounting_quat_inv[2];
  float d = mounting_quat_inv[3];
  imu_data.qw = w * a - x * b - y * c - z * d;
  imu_data.qx = x * a + w * b - z * c + y * d;
  imu_data.qy = y * a + z * b + w * c - x * d;
  imu_data.qz = z * a - y * b + x * c + w * d;
}

void IMUService::pubMsg() {
  // control_msgs::imu_data msg;
  // msg.angular_velocity.x = imu_data.vx;
  // msg.angular_velocity.y = imu_data.vy;
  // msg.angular_velocity.z = imu_data.vz;
  // msg.linear_acceleration.x = imu_data.ax;
  // msg.linear_acceleration.y = imu_data.ay;
  // msg.linear_acceleration.z = imu_data.az;
  // msg.orientation.x = imu_data.qx;
  // msg.orientation.y = imu_data.qy;
  // msg.orientation.z = imu_data.qz;
  // msg.orientation.w = imu_data.qw;
  // infoPub.publish(msg);
}

bool IMUService::init() {
  // mounting_matrix = {
  //     0, 0, -1,
  //     1, 0, 0,
  //     0, 1, 0
  // };
  mounting_matrix = {0, 1, 0, -1, 0, 0, 0, 0, 1};
  mounting_quat_inv = {0.7071, 0, 0, 0.7071};

  // int argc;
  int a{0};
  // ros::init(argc, NULL, "imu_info_publisher");
  // ros::init(a, NULL, "imu_info_publisher");
  // ros::NodeHandle hdl;

  // infoPub = hdl.advertise<control_msgs::imu_data>("/imu_data", 10);
  ser = new serial::Serial;

  if (!ser->isOpen()) {
    ser->setPort(IMU_DEV_NAME);
    LDEBUG(LDEBUG_NOTICE, "Serial Device %s\n", IMU_DEV_NAME.c_str());

    ser->setBaudrate(IMU_SERIAL_BAUDRATE);
    LDEBUG(LDEBUG_NOTICE, "Set Baudrate to %d\n", IMU_SERIAL_BAUDRATE);

    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser->setTimeout(to);

    ser->open();
    LDEBUG(LDEBUG_NOTICE, "Serial init finish.\n");
  }

  if (!ser->isOpen()) {
    LDEBUG(LDEBUG_ERR, "Serial init failed.\n");
    return false;
  }

  gettimeofday(&lastTime, NULL);
  return true;
}

IMU_DATA IMUService::getData() { return imu_data; }

void IMUService::procOnce(IMU_TYPE type) {
  uint32_t readSize = ser->available();
  uint8_t readBuf[readSize];

  try {
    ser->read(readBuf, readSize);
  } catch (exception &e) {
    LDEBUG(LDEBUG_ERR, "[imu_service] %s\n", e.what());
    ser->close();
    ser->open();
    if (!ser->isOpen()) {
      LDEBUG(LDEBUG_ERR, "[imu_service] serial port reopen failed.\n");
      return;
    }
  }
  // LDEBUG(LDEBUG_NOTICE, "Receive %d bytes data.\n", readSize);

  //判断缓存区大小
  if (dataIndex + readSize > kTatalBufSize) {
    LDEBUG(LDEBUG_ERR, "read size out !!! \n");
    return;
  }

  memcpy(totalBuf + dataIndex, readBuf, readSize);
  dataIndex += readSize;

  uint needLen = (type == IMU_F5101NAC) ? 34 : 67;
  if (dataIndex >= needLen)  // IMU发送数据长度
  {
    for (int i = 0; i < dataIndex; i++) {
      if (dataIndex - i >= needLen && checkHeader(&totalBuf[i], type)) {
        // Check the checksum
        if (checkCRC(&totalBuf[i], type))
          parseData(&totalBuf[i], type);
        else {
          errCnt++;

          sendTopicIndex = i;
          continue;
        }

        i += needLen - 1;
        sendTopicIndex = i;

        // pubMsg();
        fpsCnt++;
        gettimeofday(&recvTime, NULL);
        // spinor
#ifndef __LDEBUG
        if (timeval_diff(&recvTime, &lastTime) >= 1000.0) {
          LDEBUG(LDEBUG_NOTICE,
                 "[TimeStamps=%ld.%ld fpsCnt =%d readSize =%d errCnt=%d] \n\
                        X               Y               Z               W  \n \
                   V:   %f      %f      %f \n\
                   A:   %f      %f      %f \n\
                   Q:   %f      %f      %f      %f\n",
                 recvTime.tv_sec, recvTime.tv_usec, fpsCnt, readSize, errCnt, imu_data.vx, imu_data.vy, imu_data.vz, imu_data.ax, imu_data.ay,
                 imu_data.az, imu_data.qx, imu_data.qy, imu_data.qz, imu_data.qw);
          fpsCnt = 0;
          lastTime.tv_sec = recvTime.tv_sec;
          lastTime.tv_usec = recvTime.tv_usec;
        }
#endif
      }
    }
    //删除已读数
    if (sendTopicIndex >= 0) {
      dataIndex = dataIndex - sendTopicIndex - 1;
      for (int i = 0; i < dataIndex; i++) {
        totalBuf[i] = totalBuf[sendTopicIndex + 1 + i];
      }
      sendTopicIndex = -1;
    }
  }
}
