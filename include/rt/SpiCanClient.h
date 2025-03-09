/*************************************************
Copyright(c) 2024 Linxai Tech
Author: spinor zhu
Date: 2024-01-10
Description: Ethernet-can communication interface class
**************************************************/
#ifndef _rt_SpiCanClient
#define _rt_SpiCanClient

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#include <thread>

#include "comm.h"

using namespace std;

#define BUF_SIZE 88
#define FRAME_SIZE 13
#define __packed __attribute__((packed))

struct FRAME {
  uint8_t info;
  uint32_t id;
  uint64_t data;
} __packed;

typedef int (*callback)(uint8_t *, uint, void *contex, int idx);  //定义回调函数原型

class SpiCanClient {
 private:
  int sockFD;
  uint cliPort, srvPort;
  sockaddr_in cliAddr, srvAddr;
  socklen_t srvAddrLen;
  uint8_t mbuf[BUF_SIZE];
  timeval recvTime, sendTime;
  FRAME frm;
  thread thdRecv;

  callback callback_ = nullptr;  //记录函数指针，用于调用
  void *contex_ = nullptr;       //记录classA的指针，用于传回给classA
  int idx__;

  uint64_t htonll(uint64_t val) { return (((uint64_t)htonl(val)) << 32) + htonl(val >> 32); }

  void recvPkt();
  void prodPkt(uint RTR, uint IDE, uint canID, uint8_t *data, uint dlc);
  void procPkt(uint8_t *buf, uint len);  // Deal with the received packet, do call back

 public:
  SpiCanClient() {}

  int init(uint cPort, char *cAddr, uint sPort, char *sAddr);
  void listen();
  void sendPkt(uint RTR, uint IDE, uint canID, uint8_t *data, uint dlc);
  void regCallback(callback callback, void *contexa, int idx);

  ~SpiCanClient() {
    if (thdRecv.joinable()) thdRecv.join();

    //关闭套接字
    close(sockFD);
  }
};
#endif