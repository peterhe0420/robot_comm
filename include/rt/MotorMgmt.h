/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: Motor management, it provides API interface to MC (Motion control)
module
**************************************************/
#include "CanOpenProt.h"
#include "CanProt.h"

#define LOCALADDR "192.168.1.203"  //设置本地IP地址
#define SERVADDR "192.168.1.5"     //设置设备IP地址

const uint LOCALPORT[4] = {9998, 9997, 9996, 9995};  //本地端口号
const uint SERVPORT[4] = {5000, 5100, 5200, 5300};   //设备端口号
// const double MOTOR_MAX_POS  = 45.0;
// const double MOTOR_MAX_VEL  = 45.0;
// const double MOTOR_MAX_TOR  = 20.0;
// const double MOTOR_MAX_KP  = 500;
// const double MOTOR_MAX_KD  = 5;
const uint CAN_MASTER_ID = 0x0;  // 0x700;
const uint CAN_MOTOR_ID = 0x0;   // 0x200;
const uint ID_ANODE = 0x1;
const uint ID_HNODE = 0x2;

// #define __MOTOR_NODEBUG

enum MotorPosition {
  MotorPos_FrontLeft = 0,
  MotorPos_FrontRight,
  MotorPos_RearLeft,
  MotorPos_RearRight,
  MotorPos_MAX,
};

enum MotorType {
  MotorType_ANode = 0,
  MotorType_HNode,
  MotorType_KNode,
  MotorType_MAX,
};

const uint idx_from = MotorPos_FrontLeft;
const uint idx_to = MotorPos_RearRight;

typedef int (*callbackMC)(MotorParamGet param, MotorPosition pos, MotorType type, void *contex);

class MotorMgmt {
 private:
  // int motorPos;   // front left, front right, rear left, rear right
  // int motorType;  // A, H, K node

  callbackMC callback_ = nullptr;  //记录函数指针，用于调用
  void *contex_ = nullptr;         //记录classA的指针，用于传回给classA

  // UdpCanClient udpConn[MotorPos_MAX];
  SpiCanClient spiConn[MotorPos_MAX];
  // UdpCanClient sendConn[MotorPos_MAX];

  static int recvCallBack(uint8_t *buf, uint len, void *contex, int idx);

 public:
  CanProt MotorNodeA[MotorPos_MAX];
  CanProt MotorNodeH[MotorPos_MAX];
  CanOpenProt MotorNodeK[MotorPos_MAX];

  MotorMgmt() {}

  ~MotorMgmt() {}

  int initConn();
  void initMotor();
  void setMotorMode();
  void setMotorPara(MotorPosition pos, MotorType type, MotorParamSet param);
  void setZeroCalib(MotorPosition pos, MotorType type);
  void getMotorPara(MotorPosition &pos, MotorType &type, MotorParamGet &param);
  void regCallback(callbackMC callback, void *contexa);
};