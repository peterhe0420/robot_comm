#include "rt/MotorMgmt.h"

#include "lcm-types/cpp/spi_command_t.hpp"
#include "lcm-types/cpp/spi_data_t.hpp"

extern void spi_send_receive(spi_command_t *command, spi_data_t *data);
int MotorMgmt::recvCallBack(uint8_t *buf, uint len, void *contex, int idx) {
  MotorMgmt *tmp = (MotorMgmt *)contex;
  LDEBUG(LDEBUG_INFO, "Callback: len - %d\n", len);
  for (int i = 0; i < len; ++i) LDEBUG(LDEBUG_INFO, "%02X ", (uint8_t)buf[i]);
  LDEBUG(LDEBUG_INFO, "\n");

  uint dlc = buf[0] & 0xf;
  if (len < 5 + dlc) {
    LDEBUG(LDEBUG_ERR, "Packet length is not right.\n");
    return -1;
  }

  if (idx >= MotorPos_MAX) {
    LDEBUG(LDEBUG_ERR, "Callback ID is not right.\n");
    return -1;
  }

  uint canID = ((uint)buf[1] << 24) | ((uint)buf[2] << 16) | ((uint)buf[3] << 8) | buf[4];
  LDEBUG(LDEBUG_INFO, "canID = 0x%X dlc = 0x%02X\n", canID, dlc);

  MotorParamGet param;
  uint type;
  if (CAN_MASTER_ID == canID) {
    uint mID = buf[5] & 0x3;

    if (ID_ANODE == mID) {
      tmp->MotorNodeA[idx].recvPkt(canID, buf + 5, len - 5);
      param = tmp->MotorNodeA[idx].getMotorParam();
      type = MotorType_ANode;
    } else if (ID_HNODE == mID) {
      tmp->MotorNodeH[idx].recvPkt(canID, buf + 5, len - 5);
      param = tmp->MotorNodeH[idx].getMotorParam();
      type = MotorType_HNode;
    }
  } else {
    tmp->MotorNodeK[idx].recvPkt(canID, buf + 5, len - 5);
    param = tmp->MotorNodeK[idx].getMotorParam();
    type = MotorType_KNode;
    if (canID != (NODE_ID + CANID_RPDO1)) return 0;
  }

  if (tmp->callback_) tmp->callback_(param, (MotorPosition)idx, (MotorType)type, contex);

  return 0;
}

void MotorMgmt::setZeroCalib(MotorPosition pos, MotorType type) {
  if (pos < 0 || pos >= MotorPos_MAX) {
    LDEBUG(LDEBUG_ERR, "Motor position para is not right.\n");
    return;
  }
  switch (type) {
    case MotorType_ANode:
      MotorNodeA[pos].calibZero();
      break;
    case MotorType_HNode:
      MotorNodeH[pos].calibZero();
      break;
    case MotorType_KNode:
      MotorNodeK[pos].calibZero();
      break;
    default:
      break;
  }
}

void MotorMgmt::setMotorPara(MotorPosition pos, MotorType type, MotorParamSet param) {
  if (pos < 0 || pos >= MotorPos_MAX) {
    LDEBUG(LDEBUG_ERR, "Motor position para is not right.\n");
    return;
  }
  switch (type) {
    case MotorType_ANode:
      MotorNodeA[pos].sendCtrlPkt(param.pos, param.vel, param.torque, param.kp, param.kd);
      break;
    case MotorType_HNode:
      MotorNodeH[pos].sendCtrlPkt(param.pos, param.vel, param.torque, param.kp, param.kd);
      break;
    case MotorType_KNode:
      MotorNodeK[pos].sendToque(param.torque);
      break;
    default:
      break;
  }
}

void MotorMgmt::getMotorPara(MotorPosition &pos, MotorType &type, MotorParamGet &param) {}

int MotorMgmt::initConn() {
  for (int i = idx_from; i <= idx_to; ++i) {
    int ret = spiConn[i].init(LOCALPORT[i], (char *)LOCALADDR, SERVPORT[i], (char *)SERVADDR);
    if (LERRCODE_OK != ret) {
      return ret;
    };

    spiConn[i].regCallback(MotorMgmt::recvCallBack, this, i);
    spiConn[i].listen();
  }

  return LERRCODE_OK;
}

void MotorMgmt::initMotor() {
  extern spi_command_t spi_command_drv;
  extern spi_data_t spi_data_drv;

  spi_command_drv.flags[1] = 0xFEFE;
  spi_command_drv.flags[3] = 0xFEFE;
  spi_send_receive(&spi_command_drv, &spi_data_drv);
  usleep(2000000);

  spi_command_drv.flags[0] = 1;  // 0xFC;
  spi_command_drv.flags[1] = 1;  // 0xFC;
  spi_command_drv.flags[2] = 1;  // 0xFC;
  spi_command_drv.flags[3] = 1;  // 0xFC;
  spi_send_receive(&spi_command_drv, &spi_data_drv);
}
void MotorMgmt::setMotorMode() {
  extern spi_command_t spi_command_drv;
  extern spi_data_t spi_data_drv;
  spi_command_drv.flags[0] = 0xEC;
  spi_command_drv.flags[2] = 0xEC;
  spi_send_receive(&spi_command_drv, &spi_data_drv);
}
void MotorMgmt::regCallback(callbackMC callback, void *contex) {
  callback_ = callback;
  contex_ = contex;
}