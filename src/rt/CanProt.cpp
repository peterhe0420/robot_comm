#include "rt/CanProt.h"

#include "rt/rt_spi.h"

void CanProt::sendCtrlPkt(double p, double v, double t, double kp, double kd) {
  uint8_t buf[8];
  uint cp = double2uint(p, paraThreshold.posMin, paraThreshold.posMax, 15);
  uint cv = double2uint(v, paraThreshold.velMin, paraThreshold.velMax, 12);
  uint ct = double2uint(t, paraThreshold.tauMin, paraThreshold.tauMax, 12);
  uint ckp = double2uint(kp, paraThreshold.kpMin, paraThreshold.kpMax, 12);
  uint ckd = double2uint(kd, paraThreshold.kdMin, paraThreshold.kdMax, 12);

  buf[0] = cp >> 8 & 0x7f;
  buf[1] = cp & 0xff;
  buf[2] = cv >> 4;
  buf[3] = (cv & 0xf) << 4 | (ckp >> 8);
  buf[4] = ckp & 0xff;
  buf[5] = ckd >> 4;
  buf[6] = (ckd & 0xf) << 4 | (ct >> 8);
  buf[7] = ct & 0xff;

  sendCanPkt(motorID, buf, 8);
}
void CanProt::cancmddataprepare() {
  uint8_t buf[8] = {0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, MOTOR_STRAT};
  go(i, 0, 2)

      sendCanPkt(motorID, buf, 8);
  // spi_to_spine();

  // spi_send_receive(,);
}
void CanProt::enterMotor() {
  // uint8_t buf[8] = {0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, MOTOR_STRAT};
  // go(i, 0, 2)

  // sendCanPkt(motorID, buf, 8);
  // spi_to_spine();

  // extern spi_command_t spi_command_drv;
  // extern spi_data_t spi_data_drv;
  // go(i,0,4)
  // {
  //     spi_command_drv.flags[i]=0xFC;
  // }
  // spi_send_receive(&spi_command_drv,&spi_data_drv);
}

// MotorParamGet CanProt::MotorParam_callback(spine_data_t *spine_data) {
//     param.pos = *spine_data->q_abad;
//     param.temp = *spine_data->qd_abad ;

//     return param;
// //     param.torque = *spine_data ;
// //     param.vel = ;
// }

void CanProt::exitMotor() {
  uint8_t buf[8] = {0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, MOTOR_STOP};
  go(i, 0, 2) sendCanPkt(motorID, buf, 8);
}

void CanProt::calibZero() {
  uint8_t buf[8] = {0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, MOTOR_ANGLE_ZERO};
  go(i, 0, 2) sendCanPkt(motorID, buf, 8);
}

void CanProt::changeID() {
  uint8_t buf[8] = {0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, MOTOR_CHANGE_ID};
  sendCanPkt(motorID, buf, 8);
}

void CanProt::sendGetTempCmd() { writeCmd(0, 0, MOTOR_OR_temperature); }

void CanProt::setMode() {
  // go(i, 0, 2) writeCmd(type, 1, MOTOR_WR_CONTROL_MODE);

  // extern spi_command_t spi_command_drv;
  // extern spi_data_t spi_data_drv;
  // // go(i,0,3)
  // // {

  // // }
  // spi_send_receive(&spi_command_drv,&spi_data_drv);
}

void CanProt::writeCmd(float parameter, uint8_t RW, uint8_t type) {
  unsigned char *pdata = (unsigned char *)&parameter;
  uint8_t buf[8] = {0x80, *pdata++, *pdata++, *pdata++, *pdata++, RW, type, 0xEC};  //小端模式
  sendCanPkt(motorID, buf, 8);
}

void CanProt::initCanID(uint masterID, uint motorID) {
  this->masterID = masterID;
  this->motorID = motorID;
}

void CanProt::initPara(MotorThreshold t) { memcpy(&paraThreshold, &t, sizeof t); }

// void CanProt::sendCanPkt(uint canID, uint8_t* buf, uint len)  {
//     spi_command_t cmd;
//     spi_data_t data;
//       memset(&cmd, 0, sizeof(cmd));
//   memset(&data, 0, sizeof(data));

//      spi_send_receive(&cmd,&data);
// }

void CanProt::recvPkt(uint canID, uint8_t *buf, uint len) {
  static int temp_cnt = 0;
  temp_cnt++;
  if (0 == temp_cnt % 500) {
    sendGetTempCmd();
    temp_cnt = 0;
  }

  if (canID == masterID) {
    if (buf[0] & 0x80) {      // command feedback
      uint8_t type = buf[5];  //电机返回的是哪个参数
      if (MOTOR_OR_temperature == type) {
        typedef union {
          uint8_t uValue[4];
          float fValue;
        } unionFloat;

        unionFloat canRecev;
        canRecev.uValue[0] = buf[1];  // canRecev.fValue中为返回的对应参数的值
        canRecev.uValue[1] = buf[2];
        canRecev.uValue[2] = buf[3];
        canRecev.uValue[3] = buf[4];

        temperature = canRecev.fValue;
        param.temp = temperature;
        if (temperature >= MOTOR_TEMPERATOR_GATEWAY) {
          LDEBUG(LDEBUG_CRI, "Temperature (%0.1f) exceeds the limitation.\n", temperature);
          exitMotor();
        }
      }
    } else {
      uint pos = ((uint)buf[1] << 8) | buf[2];
      uint vel = ((uint)buf[3] << 4) | (buf[4] >> 4);
      uint tau = (((uint)buf[4] & 0xf) << 8) | buf[5];

      param.pos = uint2double(pos, paraThreshold.posMin, paraThreshold.posMax, 16);
      param.vel = uint2double(vel, paraThreshold.velMin, paraThreshold.velMax, 12);
      param.torque = uint2double(tau, paraThreshold.tauMin, paraThreshold.tauMax, 12);

      LDEBUG(LDEBUG_INFO, "%f %f %f\n", param.pos, param.vel, param.torque);
    }
  }
}