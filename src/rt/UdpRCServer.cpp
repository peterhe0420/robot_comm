/*!
 * @file UdpRCServer.cpp
 * @author spinor zhu
 * @date 2023-09-08
 * @brief Udp-rc communication
 */
#include "rt/UdpRCServer.h"

#include "common/include/printLog.h"

extern rc_data_t rc_data_rx;
rc_to_lcm_T rc2lcm_cmd;

int32_t update_rc;

int UdpRCServer::init(uint cPort, char *cAddr, uint sPort, char *sAddr) {
  // 建立套接字: IPV4,数据报套接字类型,不指定协议
  sockFD = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockFD == -1) {
    cerr << "Failed to create socket" << endl;
    return LERRCODE_SOCK_CREATE_FAILED;
  }
  LOG_INFO("\n**[UDP RC SERVER]socket FD OK\n");

  srvPort = sPort;

  memset(&srvAddr, 0, sizeof(srvAddr));
  srvAddr.sin_family = AF_INET;
  srvAddr.sin_port = htons(sPort);
  srvAddr.sin_addr.s_addr = inet_addr(sAddr);

  // std::cout << "UDP RC SERVER " << inet_ntoa(srvAddr.sin_addr) << ":" << ntohs(srvAddr.sin_port) << std::endl;

  // LOG_INFO("\n**[UDP RC SERVER] sin_port=%d,sin_addr=%d\n",srvAddr.sin_port,srvAddr.sin_addr.s_addr );

  cliPort = cPort;
  memset(&cliAddr, 0, sizeof(cliAddr));
  cliAddr.sin_family = AF_INET;                // 协议类型IPV4
  cliAddr.sin_port = htons(cPort);             // 端口号-网络字节序
  cliAddr.sin_addr.s_addr = inet_addr(cAddr);  // IP地址

  // LDEBUG(LDEBUG_NOTICE, "Bind server address: [%d] %d\n", srvAddr.sin_port, srvAddr.sin_addr.s_addr);
  int ret = bind(sockFD, (struct sockaddr *)&srvAddr, sizeof(srvAddr));
  if (ret < 0) {
    // LDEBUG(LDEBUG_ERR, "Bind server address error: [%d] %d\n", srvAddr.sin_port, srvAddr.sin_addr.s_addr);
    LOG_INFO("\n**[UDP RC SERVER]bind ret =  %d  **\n", ret);
    close(sockFD);
    return LERRCODE_IP_BINGING_FAILED;
  }

  LOG_INFO("\n**[UDP RC SERVER]bind   OK**\n");

  return LERRCODE_OK;
}

void UdpRCServer::procPkt(uint8_t *buf, uint len) {
  for (int i = 0; i < len; i++) LDEBUG(LDEBUG_INFO, "%02X ", (uint8_t)buf[i]);
  LDEBUG(LDEBUG_INFO, "\n");
  if (callback_) callback_(buf, len, contex_, idx__);
}
void UdpRCServer::recvPkt() {
  pthread_setname_np(pthread_self(), "rc_udprecv");
  // cout << "Thread in listen: " << this_thread::get_id() << endl;

  // struct sockaddr_in tmpAddr;
  socklen_t cliAddrLen = sizeof(cliAddr);

  while (true) {
    // 清零初始化
    bzero(mbuf, BUF_SIZE);
    int recv_size = recvfrom(sockFD, mbuf, sizeof(mbuf), 0, (struct sockaddr *)&cliAddr, &cliAddrLen);
    // gettimeofday(&recvTime, NULL);
    // LDEBUG(LDEBUG_INFO, "\nRecv pkt time: %ld.%ld size: %d from [%s - %d]\n", recvTime.tv_sec, recvTime.tv_usec,
    // recv_size,
    //        inet_ntoa(cliAddr.sin_addr), ntohs(cliAddr.sin_port));

    if (recv_size == -1) {
      std::cerr << "Error receiving data: " << strerror(errno) << std::endl;
      break;
    }
    // std::cout << "Received from " << inet_ntoa(cliAddr.sin_addr) << ":" << ntohs(cliAddr.sin_port) <<
    // "recv_size:" << recv_size << "mbuf:" << mbuf
    //           << std::endl;
    rc_data_rx.skydroid[0] = mbuf[0];
    rc_data_rx.skydroid[1] = mbuf[1];
    rc_data_rx.skydroid[2] = mbuf[2];
    rc_data_rx.skydroid[3] = mbuf[3];
    rc_data_rx.skydroid[4] = mbuf[4];
    rc_data_rx.skydroid[5] = mbuf[5];
    rc_data_rx.skydroid[6] = mbuf[6];
    rc_data_rx.skydroid[7] = mbuf[7];
    rc_data_rx.skydroid[8] = mbuf[8];
    rc_data_rx.b1 = mbuf[9];
    rc_data_rx.rc_data_len = mbuf[10];
    rc_data_rx.rocker_x[1] = mbuf[11] << 8 | mbuf[12];
    rc_data_rx.rocker_y[1] = mbuf[13] << 8 | mbuf[14];
    rc_data_rx.rocker_y[0] = mbuf[15] << 8 | mbuf[16];
    rc_data_rx.rocker_x[0] = mbuf[17] << 8 | mbuf[18];
    rc_data_rx.SW[0] = mbuf[19] << 8 | mbuf[20];
    rc_data_rx.SW[1] = mbuf[21] << 8 | mbuf[22];
    rc_data_rx.SW[2] = mbuf[23] << 8 | mbuf[24];
    rc_data_rx.KEY_ABCDEF = mbuf[27] << 8 | mbuf[28];
    rc_data_rx.KEY_Light = mbuf[29] << 8 | mbuf[30];
    rc_data_rx.AUX[0] = mbuf[31] << 8 | mbuf[32];
    rc_data_rx.AUX[1] = mbuf[33] << 8 | mbuf[34];
    rc_data_rx.rocker_x[2] = mbuf[35] << 8 | mbuf[36];
    rc_data_rx.rocker_y[2] = mbuf[37] << 8 | mbuf[38];
    rc_data_rx.reserved[0] = mbuf[39];
    rc_data_rx.reserved[1] = mbuf[40];
    rc_data_rx.reserved[2] = mbuf[41];
    rc_data_rx.reserved[3] = mbuf[42];

    // checksum
    //   1
    rc_data_rx.checksum_u8 = mbuf[43];

    rc2lcm_cmd.Right_X2 = 1.0f * (rc_data_rx.rocker_x[1] - 1002) / 720;   // yaw角
    rc2lcm_cmd.Right_Y2 = -1.0f * (rc_data_rx.rocker_y[1] - 1002) / 720;  // -1.0f: SWAP UP-DOWM
    rc2lcm_cmd.Left_Y1 = 1.0f * (rc_data_rx.rocker_y[0] - 1002) / 720;    //横向//横向线速度：y*max_linear_y [-1, 1]
    rc2lcm_cmd.Left_X1 = 1.0f * (rc_data_rx.rocker_x[0] - 1002) / 720;    //纵向    //纵向线速度：x*max_linear_x [-1, 1]
    if (rc_data_rx.SW[0] == 282) {
      rc2lcm_cmd.SW[0] = 1;
    } else if (rc_data_rx.SW[0] == 1002) {
      rc2lcm_cmd.SW[0] = 2;
    } else if (rc_data_rx.SW[0] == 1722) {
      rc2lcm_cmd.SW[0] = 3;
    }

    if (rc_data_rx.SW[1] == 282) {
      rc2lcm_cmd.SW[1] = 1;
    } else if (rc_data_rx.SW[1] == 1002) {
      rc2lcm_cmd.SW[1] = 2;
    } else if (rc_data_rx.SW[1] == 1722) {
      rc2lcm_cmd.SW[1] = 3;
    }

    if (rc_data_rx.SW[2] == 282) {
      rc2lcm_cmd.SW[2] = 1;
    } else if (rc_data_rx.SW[2] == 1002) {
      rc2lcm_cmd.SW[2] = 2;
    } else if (rc_data_rx.SW[2] == 1722) {
      rc2lcm_cmd.SW[2] = 3;
    }
    rc_data_rx.SW[3] = mbuf[25] << 8 | mbuf[26];
    if (rc_data_rx.SW[3] == 282) {
      rc2lcm_cmd.SW[3] = 1;
    } else if (rc_data_rx.SW[3] == 1002) {
      rc2lcm_cmd.SW[3] = 2;
    } else if (rc_data_rx.SW[3] == 1722) {
      rc2lcm_cmd.SW[3] = 3;
    }

    // 6segment-SW ABDEF
    if (rc_data_rx.KEY_ABCDEF == 426) {
      rc2lcm_cmd.KEY_ABCDEF = KEY_A;
    } else if (rc_data_rx.KEY_ABCDEF == 685) {
      rc2lcm_cmd.KEY_ABCDEF = KEY_B;
    } else if (rc_data_rx.KEY_ABCDEF == 858) {
      rc2lcm_cmd.KEY_ABCDEF = KEY_C;
    } else if (rc_data_rx.KEY_ABCDEF == 1074) {
      rc2lcm_cmd.KEY_ABCDEF = KEY_D;
    } else if (rc_data_rx.KEY_ABCDEF == 1290) {
      rc2lcm_cmd.KEY_ABCDEF = KEY_E;
    } else if (rc_data_rx.KEY_ABCDEF == 1578) {
      rc2lcm_cmd.KEY_ABCDEF = KEY_F;
    }
    // Light
    rc2lcm_cmd.KEY_Light = (int32_t)1.0f * (rc_data_rx.KEY_Light - 1002) / 720;  //[-1,1]

    rc2lcm_cmd.AUX1 = 1.0f * (rc_data_rx.AUX[0] - 1002) / 720;

    rc2lcm_cmd.AUX2 = 1.0f * (rc_data_rx.AUX[1] - 1002) / 720;
    // mini-X3 mini-y3
    //  2        2

    rc2lcm_cmd.Left_X3 = 1.0f * (rc_data_rx.rocker_x[2] - 1002) / 720;

    rc2lcm_cmd.Left_Y3 = 1.0f * (rc_data_rx.rocker_y[2] - 1002) / 720;

    // res
    //  4

    // RC USED MESSAGE
    // LOG_INFO(
    //     "*****rc2lcm_cmd will be published******\n\
    //     Right_X2            = %f;\n\
    //     Left_Y1             = %f;\n\
    //     Left_X1             = %f;\n\
    //     SW[1]               = %d;\n\
    //     KEY_ABCDEF          = %d;\n\
    //     AUX1                = %f;\n\
    //     AUX2                = %f;\n",
    //     rc2lcm_cmd.Right_X2, rc2lcm_cmd.Left_Y1, rc2lcm_cmd.Left_X1, rc2lcm_cmd.SW[1], rc2lcm_cmd.KEY_ABCDEF,
    //     rc2lcm_cmd.AUX1, rc2lcm_cmd.AUX2);
    // LOG_INFO(
    //     "\
    //     Right_Y2            = %f;\n\
    //     Left_Y3             = %f;\n\
    //     Left_X3             = %f;\n\
    //     SW[0]               = %d;\n\
    //     SW[2]               = %d;\n\
    //     SW[3]               = %d;\n\
    //     KEY_Light           = %d;\n",
    //     rc2lcm_cmd.Right_Y2, rc2lcm_cmd.Left_Y3, rc2lcm_cmd.Left_X3, rc2lcm_cmd.SW[0], rc2lcm_cmd.SW[2],
    //     rc2lcm_cmd.SW[3], rc2lcm_cmd.KEY_Light);

    // // rockers[0-2]
    // LOG_INFO("\n****udp rc data for test****\n");
    // LOG_INFO("X1 = %d; Y1 = %d; X2 = %d; Y2 = %d; X3 = %d; Y3 = %d;\n", rc_data_rx.rocker_x[0],
    // rc_data_rx.rocker_y[0], rc_data_rx.rocker_x[1],
    //        rc_data_rx.rocker_y[1], rc_data_rx.rocker_x[2], rc_data_rx.rocker_y[2]);
    // // SW[1-4]
    // LOG_INFO("SW1 = %d;   SW2 = %d;   SW3 = %d;   SW4 = %d;\n", rc_data_rx.SW[0], rc_data_rx.SW[1],
    // rc_data_rx.SW[2], rc_data_rx.SW[3]);
    // // AUX[1-2]
    // LOG_INFO("AUX1 = %d;  AUX2 = %d;\n", rc_data_rx.AUX[0], rc_data_rx.AUX[1]);
    // LOG_INFO("KEY_ABCDEF = %d; \n", rc_data_rx.KEY_ABCDEF);
    // // KEY_Light
    // LOG_INFO("KEY_Light = %d; \n", rc_data_rx.KEY_Light);

    update_rc = 50;
    
    usleep(5000);
  }
  close(sockFD);
}
void UdpRCServer::listen() {
  thdRecv = thread(&UdpRCServer::recvPkt, this);
  // auto handle = thdRecv.native_handle();  //获取原生句柄
  // pthread_setname_np(handle, "rc_udprecv");
}

void UdpRCServer::regCallback(callback callback, void *contex, int idx) {
  callback_ = callback;
  contex_ = contex;
  idx__ = idx;
}
