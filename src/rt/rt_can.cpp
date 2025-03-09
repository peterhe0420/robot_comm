#include "rt/rt_can.h"

CanComm::CanComm(char *canName) {
  //建立套接字
  sockFD = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockFD == -1) {
    cerr << "Failed to create socket" << endl;
    return;
  }

  strcpy(ifr.ifr_name, canName);
  ioctl(sockFD, SIOCGIFINDEX, &ifr);  // define can device
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // bind socket to can interface
  if (bind(sockFD, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    cerr << "Bind can error: [" << canName << "]" << endl;
    return;
  }

  // forbid filter rule, not receive packet, only send
  // setsockopt(sockFD, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
}

void CanComm::listen() {
  thdRecv = thread(&CanComm::recvPkt, this);
  cout << "Thead Id in listen:" << this_thread::get_id() << endl;
}

void CanComm::recvPkt() {
  LDEBUG(LDEBUG_NOTICE, "Begin to listen.\n");
  while (true) {
    //清零初始化
    bzero(&frm[1], sizeof frm[1]);

    int recv_size = read(sockFD, &frm[1], sizeof frm[1]);
    // display the packet
    if (recv_size > 0) {
      LDEBUG(LDEBUG_NOTICE, "ID=0x%X DLC=%d data[0]=0x%X\n", frm[1].can_id, frm[1].can_dlc, frm[1].data[0]);
    }

    gettimeofday(&recvTime, NULL);
    LDEBUG(LDEBUG_NOTICE, "Recv pkt time: %ld.%ld\n", recvTime.tv_sec, recvTime.tv_usec);

    procPkt((char *)&frm[1], recv_size);

    if (callback_) callback_((uint8_t *)&frm[1], recv_size, contex_);
  }
}

void CanComm::prodPkt(int RTR, int IDE, int canID, u_char *data, int dlc) {
  if (dlc > CAN_MAX_DLEN) dlc = CAN_MAX_DLEN;

  frm[0].can_id = canID;
  frm[0].can_dlc = dlc;
  memcpy(frm[0].data, data, dlc);
}

void CanComm::sendPkt(int RTR, int IDE, int canID, u_char *data, int dlc) {
  prodPkt(RTR, IDE, canID, data, dlc);

  for (int i = 0; i < sizeof frm[0]; i++) LDEBUG(LDEBUG_NOTICE, "%02X ", ((u_char *)&frm[0])[i]);
  LDEBUG(LDEBUG_NOTICE, "\n");

  gettimeofday(&sendTime, NULL);
  LDEBUG(LDEBUG_NOTICE, "Send pkt time: %ld.%ld\n", sendTime.tv_sec, sendTime.tv_usec);

  int sent_bytes = write(sockFD, &frm[0], sizeof frm[0]);
  if (sent_bytes == -1) {
    cerr << "[RT CAN]Failed to send data " << endl;
    return;
  }

  // cout << "Sent " << sent_bytes << " bytes of data." << endl;
  usleep(10);
}

void CanComm::procPkt(char *buf, int len) {
  for (int i = 0; i < len; i++) LDEBUG(LDEBUG_DEBUG, "%02X ", buf[i]);
  // LDEBUG(LDEBUG_DEBUG, "\n");
}

void CanComm::regCallback(callbackCan callback, void *contex) {
  callback_ = callback;
  contex_ = contex;
}

void CanComm::setFilter(int canID) {
  rfilter.can_id = canID;
  rfilter.can_mask = CAN_SFF_MASK;
  setsockopt(sockFD, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
}