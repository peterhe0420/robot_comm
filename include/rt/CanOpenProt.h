/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: Motor can-open control protocol class
**************************************************/
#ifndef __canopenport_h_
#define __canopenport_h_
#include <math.h>

#include "CanProt.h"

#define NODE_ID 1
#define CANID_TPDO1 0x200
#define CANID_TPDO2 0x300
#define CANID_TPDO3 0x400
#define CANID_TPDO4 0x500
#define CANID_RPDO1 0x180
#define CANID_RPDO2 0x280
#define CANID_RPDO3 0x380
#define CANID_RPDO4 0x480
#define CANID_TSDO 0x600
#define CANID_RSDO 0x580
#define CANID_NMT 0x0

#define TSDO_MOD_1B 0x2F
#define TSDO_MOD_2B 0x2B
#define TSDO_MOD_4B 0x23

#define TSDO_CONTROL_WORD_ADDR 0x6040
#define TSDO_OPER_MODE_ADDR 0x6060
#define TSDO_CALIB_ZERO_METHOD_ADDR 0x6098
#define TSDO_CALIB_ZERO_OFFSET_ADDR 0x607C
#define TSDO_CALIB_ZERO_ACC_ADDR 0x609A
#define TSDO_CALIB_ZERO_SEARCH_ADDR 0x6099
#define TSDO_CALIB_ZERO_CHECK_BLOCK_ADDR 0x3565
#define TSDO_CALIB_ZERO_POS_OFFSET_ADDR 0x3634
#define TSDO_CALIB_ZERO_BLOCK_CUR_ADDR 0x3637
#define TSDO_CALIB_ZERO_TIMEOUT_ADDR 0x3643
#define TSDO_TARGET_TORQUE_ADDR 0x6071
#define TSDO_ACTUAL_TORQUE_ADDR 0x6077

#define RPDO_PARA_ADDR1 0x1400
#define RPDO_COMM_ADDR1 0x1600
#define TPDO_PARA_ADDR1 0x1800
#define TPDO_COMM_ADDR1 0x1A00
#define TPDO_PARA_ADDR2 0x1801
#define TPDO_COMM_ADDR2 0x1A01
#define TPDO_TARGET_TORQUE_ADDR 0x60710010
#define TPDO_ACTUAL_TORQUE_ADDR 0x60770010

#define CANOPEN_FPS 1
#define CANOPEN_HZ 1000

#define MOTOR_PULSE_PER_ROUND 131072
#define MOTOR_RATED_CURRENT 11309

// Unit: N*m/A
#define MOTOR_TORQUE_CONST 0.075

class CanOpenProt : public CanProt {
 private:
  volatile double actualPos;
  volatile double actualVel;
  volatile double actualTorque;

  uint getLenFromType(uint8_t type);
  uint8_t getTypeFromLen(int len);

  void sendSDO(uint canID, uint idx, uint subIdx, uint data, uint len);
  void sendPDO(uint canID, uint8_t* buf, uint len);
  void recvSDO(uint8_t* buf, uint len);
  void recvPDO(uint canID, uint8_t* buf, uint len);
  void configPDO(void);

 public:
  CanOpenProt() {}
  ~CanOpenProt() {
    // sendToque(0);
  }

  // Control driver
  void openDriver();
  void enableDriver();
  void disableDriver();
  void pauseDriver();
  void calibZero();
  void runDriver();

  // Send
  void sendToque(double t);

  // Recv
  void recvStatus();
  void recvTemp(double& t);
  void recvPVT(double& p, double& v, double& t);
  void recvPkt(uint canID, uint8_t* buf, uint len);
};
#endif