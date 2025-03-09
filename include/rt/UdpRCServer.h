/*************************************************
 *@copyright(c) 2023 Linxai Tech
 *@author: spinor zhu
 *@date: 2023-09-06
 *@description: Ethernet-can communication interface class
 **************************************************/

#ifndef __RT_UDPRCSERVER_H__
#define __RT_UDPRCSERVER_H__

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

/*
//SEND: BATTERY HEART SPEED
//UDP: LSB 1ms

//52
NUM NAME                TYPE       Bytes           notes
1   源端口              Uint16_t       2          为固定值：57379
2   目标端口            Uint16_t       2          为固定值：8011
3   帧长                Uint16_t       2          为固定值：52
4   校验码              Uint16_t       2          校验算法：BCC异或校验

//DATA 44
5   SKYDROID           Uint32_t        9          禁用(SKYDROID:)
6   未定义              Uint8_t        1  固定值0xB1，该byte之后为按键数据
7   数据长度(非UDP帧头)  Uint8_t       1             0x20 ，长度32
//RC_DATA
8   右摇杆 X2           Uint32_t       2           [282,1722]
9   右摇杆 Y2           Uint32_t       2           [282,1722]
10  左摇杆 Y1           Uint32_t       2           [282,1722]
11  左摇杆 X1           Uint32_t       2           [282,1722]

12  拨动三档开关SW1      Uint32_t      2            [282-1002-1722]
13  拨动三档开关SW2      Uint32_t      2            [282-1002-1722]
14  拨动三档开关SW3      Uint32_t      2            [282-1002-1722]
15  拨动三档开关SW4      Uint32_t      2            [282-1002-1722]

16  6段开关             Uint32_t       2  426、685、858、1074、1290、1578]

17  灯开关              Uint32_t       2           [282-1722]

18  旋钮1               Uint32_t       2           [282,1722]
19  旋钮2               Uint32_t       2           [282,1722]

20  小摇杆 X3           Uint32_t       2           [282,1722]
21  小摇杆 Y3           Uint32_t       2           [282,1722]
//
22  未定义值                /          4               /
//
23  校验码              uint8_t        1           同帧头填充校验数据方式
*/

#define BUF_SIZE 52
#define FRAME_SIZE 44
#define __packed __attribute__((packed))

#define SuLaADDR "192.168.144.3"  //设置SuLa IP地址
#define RCADDR "192.168.144.11"   //设置RC IP地址

const uint SuLaPORT = 8011;  //本地端口号
const uint RCPORT = 57379;   // RC端口号

typedef struct {
  uint16_t src_port;
  uint16_t des_port;
  uint16_t frame_len;
  uint16_t checksum_u16;

  uint8_t skydroid[9];
  uint8_t b1;
  uint8_t rc_data_len;
  uint8_t reserved[4];
  uint8_t checksum_u8;

  uint16_t rocker_x[3];
  uint16_t rocker_y[3];

  uint16_t SW[4];

  uint16_t KEY_ABCDEF;

  uint16_t KEY_Light;

  uint16_t AUX[2];
} rc_data_t;

typedef struct {
  uint16_t src_port;
  uint16_t des_port;
  uint16_t frame_len;
  uint16_t checksum_u16;

  uint8_t skydroid[9];
  uint8_t b1;
  uint8_t rc_status_len;
  uint8_t reserved[4];
  uint8_t checksum_u8;
  uint16_t speed;
  uint16_t battery;
  uint16_t heart;

} rc_status_t;

typedef struct {
  float Right_X2;
  float Left_Y1;
  float Left_X1;
  int32_t SW[4];
  int32_t KEY_ABCDEF;
  float AUX1;
  float AUX2;
  float Left_X3;
  float Left_Y3;
  float Right_Y2;
  int32_t KEY_Light;

} rc_to_lcm_T;

enum { KEY_A, KEY_B, KEY_C, KEY_D, KEY_E, KEY_F };

typedef int (*callback)(uint8_t*, uint, void* contex, int idx);  //定义回调函数原型

class UdpRCServer {
 private:
  int sockFD;
  uint cliPort, srvPort;
  sockaddr_in cliAddr, srvAddr;
  socklen_t srvAddrLen;
  uint8_t mbuf[BUF_SIZE];
  timeval recvTime, sendTime;
  // FRAME frm;
  thread thdRecv;

  callback callback_ = nullptr;  //记录函数指针，用于调用
  void* contex_ = nullptr;       //记录classA的指针，用于传回给classA
  int idx__;

  uint64_t htonll(uint64_t val) { return (((uint64_t)htonl(val)) << 32) + htonl(val >> 32); }

  void recvPkt();
  void prodPkt(uint8_t* data, uint dlc);
  void procPkt(uint8_t* buf, uint len);  // Deal with the received packet, do call back
  int recvCallBack(uint8_t* buf, uint len, void* contex);

 public:
  UdpRCServer() {}

  int init(uint cPort, char* cAddr, uint sPort, char* sAddr);
  void listen();
  void sendPkt(uint8_t* data, uint dlc);
  void regCallback(callback callback, void* contexa, int idx);

  ~UdpRCServer() {
    if (thdRecv.joinable()) thdRecv.join();

    //关闭套接字
    close(sockFD);
  }
};

#endif
