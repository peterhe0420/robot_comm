#include "rt/batMgmt.h"

#define CANNAME "can0"  //设置CAN

int batMgmt::recvCallBack(uint8_t *buf, uint len, void *contex) {
  batMgmt *tmp = (batMgmt *)contex;
  LDEBUG(LDEBUG_NOTICE, "Callback: len - %d\n", len);

  tmp->proc(buf, len);
  if (tmp->callback_) tmp->callback_(tmp->param, contex);

  return 0;
}

void batMgmt::proc(uint8_t *buf, uint len) {
  can_frame *frm = (can_frame *)buf;
  if (len < sizeof(can_frame)) return;

  for (int i = 0; i < len; i++) LDEBUG(LDEBUG_NOTICE, "%02X ", buf[i]);
  LDEBUG(LDEBUG_NOTICE, "\n");

  if (batID == frm->can_id) {
    parse(frm->data);
    // sendTempCmd();
  }
  // else if (PT100ID == frm->can_id)
  //   parsePT100(frm->data);
}

void batMgmt::parse(uint8_t *buf) {
  param.cur = ((uint16_t)buf[1] << 8) | buf[0];  // mA
  param.vol = ((uint16_t)buf[3] << 8) | buf[2];  // mV
  param.soc = buf[4];                            // percent of EQ (quantity of electric charge)
  param.temp = buf[5];                           // temprature
  param.state = ((uint16_t)buf[7] << 8) | buf[6];
}

void batMgmt::parsePT100(uint8_t *buf) {
  for (int i = 0; i < 4; i++) {
    float cur = ((uint16_t)(buf[i * 2] & 0x7f) << 8) | buf[i * 2 + 1];
    cur /= 10;
    if (0x80 & buf[i * 2]) cur = -cur;
    param.ktemp[i] = cur;
  }
}

void batMgmt::initConn() {
  conn = new CanComm((char *)CANNAME);
  // conn->setFilter(batID);
  conn->regCallback(batMgmt::recvCallBack, this);
  conn->listen();

  u_char buf[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  conn->sendPkt(0, 0, 0x0909, (u_char *)&buf, 8);
}

void batMgmt::regCallback(callbackBat callback, void *contex) {
  callback_ = callback;
  contex_ = contex;
}

// For K motor temperature process
void batMgmt::sendTempCmd() {
  u_char buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  conn->sendPkt(0, 0, 0x0301, (u_char *)&buf, 8);
}