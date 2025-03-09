/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: Can communication interface class
**************************************************/

#ifndef _rt_can
#define _rt_can

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <thread>

#include "comm.h"

using namespace std;

#define BUF_SIZE 88
typedef int (*callbackCan)(uint8_t *, uint, void *contex);  //定义回调函数原型

class CanComm {
 private:
  int sockFD;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frm[2] = {{0}};
  struct can_filter rfilter;

  char mbuf[BUF_SIZE];
  timeval recvTime, sendTime;
  thread thdRecv;

  callbackCan callback_;  //记录函数指针，用于调用
  void *contex_;          //记录classA的指针，用于传回给classA

  void recvPkt();
  void prodPkt(int RTR, int IDE, int canID, u_char *data, int dlc);

 public:
  CanComm(char *canName);
  void listen();
  void sendPkt(int RTR, int IDE, int canID, u_char *data, int dlc);
  void procPkt(char *buf, int len);
  void regCallback(callbackCan callback, void *contexa);
  void setFilter(int canID);

  ~CanComm() {
    if (thdRecv.joinable()) thdRecv.join();

    //关闭套接字
    close(sockFD);
  }
};
#endif
