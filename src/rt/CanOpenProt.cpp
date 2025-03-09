#include "rt/CanOpenProt.h"
//
uint CanOpenProt::getLenFromType(uint8_t type) {
  if (TSDO_MOD_1B == type)
    return 1;
  else if (TSDO_MOD_2B == type)
    return 2;
  else
    return 4;
}

uint8_t CanOpenProt::getTypeFromLen(int len) {
  if (1 == len)
    return TSDO_MOD_1B;
  else if (2 == len)
    return TSDO_MOD_2B;
  else
    return TSDO_MOD_4B;
}

// Control driver
void CanOpenProt::openDriver() {}

void CanOpenProt::calibZero() {
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CONTROL_WORD_ADDR, 0, 0x6, 2);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_OPER_MODE_ADDR, 0, 0x6, 2);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_METHOD_ADDR, 0, 0xfe, 1);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_OFFSET_ADDR, 0, 0x0, 4);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_ACC_ADDR, 0, 0x2aa, 4);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_SEARCH_ADDR, 1, 0x155, 4);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_SEARCH_ADDR, 2, 0x155, 4);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_CHECK_BLOCK_ADDR, 0, 0xc8, 4);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_POS_OFFSET_ADDR, 0, 0x0, 4);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_BLOCK_CUR_ADDR, 0, 0x32, 1);
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CALIB_ZERO_TIMEOUT_ADDR, 0, 0x0, 4);
}

void CanOpenProt::configPDO() {
  // Config RPDO1
  // RPDO parameter: canID 1400
  //      mapping: canID 1600
  // 1600 00 3 -> 1600 00 1
  // 1600 01 0x60400010 -> 1600 01 0x60710010
  sendSDO(NODE_ID + CANID_TSDO, RPDO_PARA_ADDR1, 2, 0xfe, 1);
  sendSDO(NODE_ID + CANID_TSDO, RPDO_COMM_ADDR1, 0, 1, 1);
  sendSDO(NODE_ID + CANID_TSDO, RPDO_COMM_ADDR1, 1, TPDO_TARGET_TORQUE_ADDR, 4);

  // Config TPDO1 (for get actual torque)
  // TPDO parameter: canID 1800
  //      mapping: canID 1A00
  // 1800 02 01  ->  1800 02 FF
  // 1800 03 1000  ->  1800 03 14
  // 1800 05 00  ->  1800 05 02
  // 1A00 00 3 -> 1A00 00 1
  // 1A00 01 0x60410010 -> 1A00 01 0x60770010
  sendSDO(NODE_ID + CANID_TSDO, TPDO_PARA_ADDR1, 2, 0xff, 1);
  sendSDO(NODE_ID + CANID_TSDO, TPDO_PARA_ADDR1, 3, CANOPEN_HZ / CANOPEN_FPS * 10, 2);
  sendSDO(NODE_ID + CANID_TSDO, TPDO_PARA_ADDR1, 5, CANOPEN_HZ / CANOPEN_FPS, 2);
  sendSDO(NODE_ID + CANID_TSDO, TPDO_COMM_ADDR1, 0, 1, 1);
  sendSDO(NODE_ID + CANID_TSDO, TPDO_COMM_ADDR1, 1, TPDO_ACTUAL_TORQUE_ADDR, 4);

  // Config TPDO2  (for get actual position and velocity)
  // TPDO parameter: canID 1801
  //      mapping: canID 1A01
  // 1801 02 0a  ->  1800 02 FF
  // 1801 03 1000  ->  1800 03 14
  // 1801 05 00  ->  1800 05 02
  sendSDO(NODE_ID + CANID_TSDO, TPDO_PARA_ADDR2, 2, 0xff, 1);
  sendSDO(NODE_ID + CANID_TSDO, TPDO_PARA_ADDR2, 3, CANOPEN_HZ / CANOPEN_FPS * 10, 2);
  sendSDO(NODE_ID + CANID_TSDO, TPDO_PARA_ADDR2, 5, CANOPEN_HZ / CANOPEN_FPS, 2);

  // Start NMT
  // canID  0, data 0101
  uint16_t data = 0x101;

  sendCanPkt(CANID_NMT, (uint8_t *)&data, 2);
}

void CanOpenProt::enableDriver() {
  configPDO();

  // need to check feedback for each command
  sendSDO(NODE_ID + CANID_TSDO, TSDO_OPER_MODE_ADDR, 0, 0x4, 1);      // Torque mode
  sendSDO(NODE_ID + CANID_TSDO, TSDO_TARGET_TORQUE_ADDR, 0, 0x0, 2);  // Default value is 0
  sendSDO(NODE_ID + CANID_TSDO, TSDO_CONTROL_WORD_ADDR, 0, 0x0f, 2);  // Enable the driver
}

void CanOpenProt::disableDriver() { sendSDO(NODE_ID + CANID_TSDO, TSDO_CONTROL_WORD_ADDR, 0, 0x06, 2); }

void CanOpenProt::runDriver() { sendSDO(NODE_ID + CANID_TSDO, TSDO_CONTROL_WORD_ADDR, 0, 0x1F, 2); }

void CanOpenProt::pauseDriver() { sendSDO(NODE_ID + CANID_TSDO, TSDO_CONTROL_WORD_ADDR, 0, 0x11F, 2); }

// Commm
void CanOpenProt::sendToque(double t) {
  int16_t targetTorque = (int16_t)(t / MOTOR_TORQUE_CONST * 1e6 / MOTOR_RATED_CURRENT);
  uint8_t buf[2];
  buf[0] = targetTorque & 0xff;
  buf[1] = targetTorque >> 8 & 0xff;

  sendPDO(NODE_ID + CANID_TPDO1, buf, 2);
}

void CanOpenProt::sendSDO(uint canID, uint idx, uint subIdx, uint data, uint len) {
  uint8_t buf[8];

  if (len > 4) len = 4;
  buf[0] = getTypeFromLen(len);
  buf[1] = idx & 0xff;
  buf[2] = idx >> 8 & 0xff;
  buf[3] = subIdx;

  for (int i = 0; i < len; i++) buf[4 + i] = (data >> (8 * i)) & 0xff;

  sendCanPkt(canID, buf, len + 4);
}

void CanOpenProt::sendPDO(uint canID, uint8_t *buf, uint len) { sendCanPkt(canID, buf, len); }

void CanOpenProt::recvStatus() {}

void CanOpenProt::recvTemp(double &t) {}

void CanOpenProt::recvPVT(double &p, double &v, double &t) {
  p = actualPos;
  v = actualVel;
  t = actualTorque;
}

void CanOpenProt::recvSDO(uint8_t *buf, uint len) {
  uint dataLen = getLenFromType(buf[0]);
  if (len < 4 + dataLen) {
    LDEBUG(LDEBUG_ERR, "SDO packet length is not right.\n");
    return;
  }
  uint idx, subIdx, data = 0;
  idx = buf[1] | ((uint)buf[2] << 8);
  subIdx = buf[3];
  for (int i = 0; i < dataLen; i++) data |= (uint)buf[4 + i] << (8 * i);
  LDEBUG(LDEBUG_INFO, "SDO idx = 0x%04X subIdx = 0x%02X data = 0x%X\n", idx, subIdx, data);
}

void CanOpenProt::recvPDO(uint canID, uint8_t *buf, uint len) {
  if (canID == (NODE_ID + CANID_RPDO1)) {
    if (len < 2) {
      LDEBUG(LDEBUG_ERR, "PDO packet length is not right. canID = 0x%X\n", canID);
      return;
    }
    int16_t curT = buf[0] | ((uint)buf[1] << 8);
    LDEBUG(LDEBUG_INFO, "torque: %d\n", curT);
    actualTorque = (double)curT * MOTOR_RATED_CURRENT / 1e6 * MOTOR_TORQUE_CONST;
    param.torque = actualTorque;
  } else if (canID == (NODE_ID + CANID_RPDO2)) {
    if (len < 8) {
      LDEBUG(LDEBUG_ERR, "PDO packet length is not right. canID = 0x%X\n", canID);
      return;
    }
    for (int i = 0; i < len; i++) LDEBUG(LDEBUG_INFO, "%02X ", (uint8_t)buf[i]);
    LDEBUG(LDEBUG_INFO, "\n");
    int32_t curP = buf[0] | ((uint)buf[1] << 8) | ((uint)buf[2] << 16) | ((uint)buf[3] << 24);
    int32_t curV = buf[4] | ((uint)buf[5] << 8) | ((uint)buf[6] << 16) | ((uint)buf[7] << 24);
    LDEBUG(LDEBUG_INFO, "pos: %d vel: %d\n", curP, curV);
    actualPos = (double)curP * 2 * PI / MOTOR_PULSE_PER_ROUND;
    actualVel = (double)curV * 2 * PI / MOTOR_PULSE_PER_ROUND;
    param.pos = actualPos;
    param.vel = actualVel;
  }
}

void CanOpenProt::recvPkt(uint canID, uint8_t *buf, uint len) {
  switch (canID) {
    case NODE_ID + CANID_RSDO:
      recvSDO(buf, len);
      break;
    case NODE_ID + CANID_RPDO1:
    case NODE_ID + CANID_RPDO2:
      recvPDO(canID, buf, len);
    default:
      break;
  }
}