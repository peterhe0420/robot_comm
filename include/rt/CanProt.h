/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: Motor can control protocol class
**************************************************/
#ifndef __CANPROT_H__
#define __CANPROT_H__

// #include "UdpCanClient.h"

#include "SpiCanClient.h"
#define MOTOR_OR_temperature 0x0D   //电机温度
#define DRIVE_OR_temperature 0x0E   //驱动温度
#define MOTOR_WR_CONTROL_MODE 0x5B  // 0-阻抗控制模式 1-速度控制模式 2-位置控制模式

#define IMPEDANCE 0
#define SPEED 1
#define POSITION 2

#define MOTOR_STRAT 0xFC
#define MOTOR_STOP 0xFD
#define MOTOR_ANGLE_ZERO 0xFE
#define MOTOR_CHANGE_ID 0xF9

#define MOTOR_TEMPERATOR_GATEWAY 75

struct MotorParamSet {
  double pos;
  double vel;
  double torque;
  double kp;
  double kd;
};

struct MotorThreshold {
  double posMax;
  double posMin;
  double velMax;
  double velMin;
  double tauMax;
  double tauMin;
  double kpMax;
  double kpMin;
  double kdMax;
  double kdMin;
};

struct MotorParamGet {
  double pos;
  double vel;
  double torque;
  float temp;
};

class CanProt {
 private:
  // CAN parameters
  uint masterID;  // CAN ID of feedback frame
  uint motorID;   // CAN ID to set the motor

  // Motor threshold parameters
  MotorThreshold paraThreshold;

  uint double2uint(double x, double x_min, double x_max, uint bits) {
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    double span = x_max - x_min;
    double base = x_min;
    return (uint)((x - base) * ((1 << bits) - 1) / span);
  }

  double uint2double(uint x, double x_min, double x_max, uint bits) {
    double span = x_max - x_min;
    double base = x_min;
    return base + x * span / ((1 << bits) - 1);
  }

  void writeCmd(float parameter, uint8_t RW, uint8_t type);

 protected:
  MotorParamGet param;
  float temperature;

 public:
  // UdpCanClient* sendConn = nullptr;
  SpiCanClient* sendConn = nullptr;

 public:
  CanProt() : temperature(-273.15) { memset(&param, 0, sizeof param); };
  ~CanProt() {}

  void sendCtrlPkt(double p, double v, double t, double kp, double kd);
  void setMode();  //(float type);
  void sendGetTempCmd();
  float getTemp() { return temperature; }
  void enterMotor();
  void cancmddataprepare();
  void exitMotor();
  void calibZero();
  void changeID();
  // void initComm(UdpCanClient* conn) { sendConn = conn; }
  void initComm(SpiCanClient* conn) { sendConn = conn; }
  void initCanID(uint masterID, uint motorID);
  void initPara(MotorThreshold t);
  void sendCanPkt(uint canID, uint8_t* buf, uint len) { sendConn->sendPkt(0, 0, canID, buf, len); }
  // { if(sendConn) sendConn->sendPkt(0, 0, canID, buf, len); }

  virtual void recvPkt(uint canID, uint8_t* buf, uint len);
  MotorParamGet getMotorParam(void) { return param; };
  // MotorParamGet MotorParam_callback(spine_data_t *spine_data);
};

#endif