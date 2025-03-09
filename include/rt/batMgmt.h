/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: Battery management module
**************************************************/

#ifndef __BATMGMT_H__
#define __BATMGMT_H__

#include <ros/ros.h>

#include "control_msgs/bat_state.h"
#include "rt_can.h"

#define CUV 0
#define OCD 1
#define SCD 2
#define DSG_OT 3
#define RCA 4
#define DSG_UT 5
#define REVC0 6
#define REVC1 7
#define COV 8
#define OCC 9
#define CHG_OT 10
#define CHG_UT 11
#define MOS_OT 12
#define VOLDIFF 13
#define TEMPDIFF 14
#define ALERT_SOCLOW 15

struct batParamGet {
  int16_t cur;
  uint16_t vol;
  u_char soc;
  char temp;
  uint16_t state;
  float ktemp[4];
};

typedef int (*callbackBat)(batParamGet param, void* contex);

class batMgmt {
 private:
  uint batID;
  uint PT100ID;
  CanComm* conn = nullptr;

  callbackBat callback_ = nullptr;  //记录函数指针，用于调用
  void* contex_ = nullptr;          //记录classA的指针，用于传回给classA

  ros::Publisher infoPub;

  static int recvCallBack(uint8_t* buf, uint len, void* contex);
  void parse(uint8_t* buf);
  void parsePT100(uint8_t* buf);

 protected:
  batParamGet param;

 public:
  batMgmt() { memset(&param, 0, sizeof param); }
  ~batMgmt() {}

  void initConn();
  void proc(uint8_t* buf, uint len);
  void initCanID(uint id1, uint id2) {  //
    batID = id1;                        //
    PT100ID = id2;
  }
  void initCanIDb(uint id1) {  //
    batID = id1;
  }
  void sendTempCmd();

  batParamGet getBatParam(void) { return param; }
  void regCallback(callbackBat callback, void* contexa);
};

#endif