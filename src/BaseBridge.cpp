#include "BaseBridge.h"
extern int32_t update_rc;
void BaseBridge::runRemoteCtrlPadCallBack() {
  // std::cout << "rcdata.KEY_ABCDEF0000:" << std::endl;
  rc_to_lcm_T* UDPrcdata = get_udprc_data();
  rc_to_lcm_t rcdata;
  gamepad_lcmt gamepad_lcmt_msg;
  while (true) {
    rcdata.Right_X2 = UDPrcdata->Right_X2;
    rcdata.Right_Y2 = UDPrcdata->Right_Y2;
    rcdata.Left_Y1 = UDPrcdata->Left_Y1;
    rcdata.Left_X1 = UDPrcdata->Left_X1;
    rcdata.Left_X3 = UDPrcdata->Left_X3;
    rcdata.Left_Y3 = UDPrcdata->Left_Y3;
    rcdata.SW[0] = UDPrcdata->SW[0];
    rcdata.SW[1] = UDPrcdata->SW[1];
    rcdata.SW[2] = UDPrcdata->SW[2];
    rcdata.SW[3] = UDPrcdata->SW[3];
    rcdata.KEY_ABCDEF = UDPrcdata->KEY_ABCDEF;
    rcdata.AUX1 = UDPrcdata->AUX1;
    rcdata.AUX2 = UDPrcdata->AUX2;
    rcdata.KEY_Light = UDPrcdata->KEY_Light;
    if (rcdata.SW[0] == 2 && update_rc) {
      if (update_rc > 0) update_rc--;
      // gamepad_lcmt _gamepad_lcmt
      gamepad_lcmt_msg.leftBumper = rcdata.SW[1];
      // gamepad_lcmt_msg.leftTriggerButton = rcdata.AUX2;
      // gamepad_lcmt_msg.rightTriggerButton = rcdata.AUX1;
      // ABCDEF:reset a b x y rtb ltb
      gamepad_lcmt_msg.a = 0;
      gamepad_lcmt_msg.b = 0;
      gamepad_lcmt_msg.x = 0;
      gamepad_lcmt_msg.y = 0;

      switch (rcdata.KEY_ABCDEF) {
        case KEY_C:
          gamepad_lcmt_msg.a = 1;
          break;
        case KEY_D:
          gamepad_lcmt_msg.b = 1;
          break;
        case KEY_E:
          gamepad_lcmt_msg.x = 1;
          break;
        case KEY_F:
          gamepad_lcmt_msg.y = 1;
          break;
        default:
          break;
      }
      // _gamepad_lcmt.leftStickButton =  ; SW3 control terrain mode
      gamepad_lcmt_msg.leftTriggerButton = 0;
      gamepad_lcmt_msg.leftTriggerAnalog = 0;
      if (rcdata.SW[2] == 1) {
        gamepad_lcmt_msg.leftTriggerButton = 1;
        gamepad_lcmt_msg.rightStickButton = 0;
        gamepad_lcmt_msg.leftStickButton = 0;

      } else if (rcdata.SW[2] == 3) {
        gamepad_lcmt_msg.leftTriggerAnalog = -0.0;
        gamepad_lcmt_msg.rightStickButton = 1;
        gamepad_lcmt_msg.leftStickButton = 0;
      } else {
        gamepad_lcmt_msg.leftTriggerAnalog = -0.0;
        gamepad_lcmt_msg.rightStickButton = 0;
        gamepad_lcmt_msg.leftStickButton = 0;
      }
      /*SW4 control impedence and passive mdoe*/
      gamepad_lcmt_msg.rightTriggerAnalog = 0;
      // gamepad_lcmt_msg.leftTriggerAnalog = 0;
      // gamepad_lcmt_msg.leftTriggerButton = 0;
      gamepad_lcmt_msg.rightTriggerButton = 0;
      if (rcdata.SW[3] == 3) {
        gamepad_lcmt_msg.rightTriggerAnalog = -1.0;
        gamepad_lcmt_msg.leftTriggerButton = 1;  // imepedence
        gamepad_lcmt_msg.rightTriggerButton = 0;
      } else if (rcdata.SW[3] == 1) {
        gamepad_lcmt_msg.rightTriggerAnalog = -1.0;
        gamepad_lcmt_msg.rightTriggerButton = 1;  // passive
        gamepad_lcmt_msg.leftTriggerButton = 0;
      } else {
      }
      // SW2 控制大腿偏置，临时给双臂四足用，非双臂四足的话可以删掉
      // gamepad_lcmt_msg.start = 0;
      // if (rcdata.SW[1] == 3) {
      //   gamepad_lcmt_msg.leftTriggerAnalog = -1;
      //   gamepad_lcmt_msg.start = 1;
      // }

      // SW2 控制模式 sw0 1 2 3  对应 sw1 2 3 4
      gamepad_lcmt_msg.start = 0;
      if (rcdata.SW[1] == 1) {  //视觉模式
        gamepad_lcmt_msg.leftStickButton = 1;
      } else if (rcdata.SW[1] == 3) {  //高速模式
        gamepad_lcmt_msg.start = 1;
      } else {  //正常模式
        ;
      }
      // gamepad_lcmt_msg.leftTriggerAnalog = rcdata.AUX1;
      // gamepad_lcmt_msg.rightTriggerAnalog = rcdata.AUX2;
      // gamepad_lcmt_msg.rightTriggerAnalog = -1.0;
      gamepad_lcmt_msg.leftStickAnalog[0] = rcdata.Left_X1;
      gamepad_lcmt_msg.leftStickAnalog[1] = rcdata.Left_Y1;
      gamepad_lcmt_msg.rightStickAnalog[0] = rcdata.Right_X2;
      gamepad_lcmt_msg.rightStickAnalog[1] = rcdata.Right_Y2;
      gamepad_cmd_ptr_->set(&gamepad_lcmt_msg);
    } else {
      if (update_rc == 0) {
        gamepad_lcmt_msg.leftStickAnalog[0] = 0.0;
        gamepad_lcmt_msg.leftStickAnalog[1] = 0.0;
        gamepad_lcmt_msg.rightStickAnalog[0] = 0.0;
        gamepad_lcmt_msg.rightStickAnalog[1] = 0.0;
        gamepad_cmd_ptr_->set(&gamepad_lcmt_msg);
      } else {
        // gamepad_lcmt_msg.rightTriggerAnalog = 0;
      }
    }
    usleep(5000);
  }
}

void BaseBridge::runGamePadCallBack() {
  rc_to_lcm_T* UDPrcdata_forros = get_udprc_data();
  while (true) {
    if (UDPrcdata_forros->SW[0] != 2) {
      gamepad_cmd_ptr_->set(gamepad_.getGamepadPtr().get());
    } else {
    }
    usleep(5000);
  }
}

bool BaseBridge::isInitialized() { return initialized_spiData_ && initialized_spiCmd_ && initialized_IMU_; }
