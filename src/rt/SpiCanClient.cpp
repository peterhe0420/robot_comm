#include "rt/SpiCanClient.h"

int SpiCanClient::init(uint cPort, char *cAddr, uint sPort, char *sAddr) {
  // 建立套接字: IPV4,数据报套接字类型,不指定协议
  sockFD = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockFD == -1) {
    cerr << "Failed to create socket" << endl;
    return LERRCODE_SOCK_CREATE_FAILED;
  }

  cliPort = cPort;
  memset(&cliAddr, 0, sizeof(cliAddr));
  cliAddr.sin_family = AF_INET;                // 协议类型IPV4
  cliAddr.sin_port = htons(cPort);             // 端口号-网络字节序
  cliAddr.sin_addr.s_addr = inet_addr(cAddr);  // IP地址

  LDEBUG(LDEBUG_NOTICE, "Bind local address: [%d] %s\n", cPort, cAddr);
  // 源地址和源端口绑定到m_socket
  if (bind(sockFD, (struct sockaddr *)&cliAddr, sizeof(cliAddr)) < 0) {
    LDEBUG(LDEBUG_ERR, "Bind local address error: [%d] %s\n", cPort, cAddr);
    return LERRCODE_IP_BINGING_FAILED;
  }

  memset(&srvAddr, 0, sizeof(srvAddr));
  srvAddr.sin_family = AF_INET;
  srvAddr.sin_port = htons(sPort);
  srvAddr.sin_addr.s_addr = inet_addr(sAddr);

  return LERRCODE_OK;
}

void SpiCanClient::listen() {
  thdRecv = thread(&SpiCanClient::recvPkt, this);
  // auto handle = thdRecv.native_handle();//获取原生句柄
  // pthread_setname_np(handle, "Motor_SpiRecv");
}

void SpiCanClient::recvPkt() {
  pthread_setname_np(pthread_self(), "motor_Spirecv");
  cout << "Thread in listen: " << this_thread::get_id() << endl;
  while (true) {
    // 清零初始化
    bzero(mbuf, BUF_SIZE);
    struct sockaddr_in tmpAddr;
    socklen_t tmpAddrLen = sizeof(tmpAddr);
    memset(&tmpAddr, 0, sizeof(tmpAddr));

    int recv_size = recvfrom(sockFD, mbuf, FRAME_SIZE, 0, (struct sockaddr *)&tmpAddr, &tmpAddrLen);
    gettimeofday(&recvTime, NULL);
    LDEBUG(LDEBUG_INFO, "\nRecv pkt time: %ld.%ld size: %d from [%s - %d]\n", recvTime.tv_sec, recvTime.tv_usec, recv_size,
           inet_ntoa(tmpAddr.sin_addr), ntohs(tmpAddr.sin_port));

    if (recv_size == FRAME_SIZE)
      procPkt(mbuf, recv_size);
    else {
      LDEBUG(LDEBUG_ERR, "Recv invalid Spi packet (size: %d) from [%s - %d]:\n", recv_size, inet_ntoa(tmpAddr.sin_addr), ntohs(tmpAddr.sin_port));
    }
  }
}

void SpiCanClient::prodPkt(uint RTR, uint IDE, uint canID, uint8_t *data, uint dlc) {
  if (dlc > 8) dlc = 8;

  frm.info = 8;  // dlc;
  if (RTR) frm.info |= 1 << 6;
  if (IDE) frm.info |= 1 << 7;
  frm.id = htonl(canID);
  frm.data = 0;
  memcpy(&frm.data, data, dlc);
}

void SpiCanClient::sendPkt(uint RTR, uint IDE, uint canID, uint8_t *data, uint dlc) {
  prodPkt(RTR, IDE, canID, data, dlc);

  for (int i = 0; i < dlc + 5; i++) LDEBUG(LDEBUG_DEBUG, "%02X ", ((uint8_t *)&frm)[i]);
  LDEBUG(LDEBUG_DEBUG, "\n");

  gettimeofday(&sendTime, NULL);
  int sent_bytes = sendto(sockFD, &frm, sizeof frm, 0, (sockaddr *)&srvAddr, sizeof(srvAddr));
  // LDEBUG(LDEBUG_DEBUG, "Send pkt time: %ld.%ld\n", sendTime.tv_sec, sendTime.tv_usec);
  if (sent_bytes == -1) {
    LDEBUG(LDEBUG_ERR, "[SPICAN]Failed to send data\n");
    return;
  }

  // LDEBUG(LDEBUG_DEBUG, "Sent %d bytes of data.\n", sent_bytes);
  // usleep(10);
}

void SpiCanClient::procPkt(uint8_t *buf, uint len) {
  for (int i = 0; i < len; i++) LDEBUG(LDEBUG_INFO, "%02X ", (uint8_t)buf[i]);
  LDEBUG(LDEBUG_INFO, "\n");
  if (callback_) callback_(buf, len, contex_, idx__);
}

void SpiCanClient::regCallback(callback callback, void *contex, int idx) {
  callback_ = callback;
  contex_ = contex;
  idx__ = idx;
}
