#ifndef BASE_BRIDGE_H
#define BASE_BRIDGE_H
#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "LS_UserParameters.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/IMUTypes.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "lcm-types/cpp/gamepad_lcmt.hpp"
#include "lcm-types/cpp/rc_to_lcm_t.hpp"
#include "lcm-types/cpp/spi_command_t.hpp"
#include "lcm-types/cpp/spi_data_t.hpp"
#include "rt/Gamepad.h"
#include "rt/rt_udprc.h"

class BaseBridge {
 public:
  BaseBridge() {
    user_params_ptr_ = std::make_shared<LS_UserParameters>();
    ctrl_params_ptr_ = std::make_shared<RobotControlParameters>();
    gamepad_cmd_ptr_ = make_shared<GamepadCommand>();
    spi_data_ptr_ = std::make_shared<spi_data_t>();
    spi_command_ptr_ = std::make_shared<spi_command_t>();
    imu_data_ptr_ = std::make_shared<VectorNavData>();
    memset(spi_data_ptr_.get(), 0, sizeof(spi_data_t));
    memset(spi_command_ptr_.get(), 0, sizeof(spi_command_t));
  };
  // ~BaseBridge(){};
  virtual void init() = 0;
  virtual void run() = 0;
  bool isInitialized();
  void runRemoteCtrlPadCallBack();
  void runGamePadCallBack();
  shared_ptr<LS_UserParameters> getUserParamsPtr() { return user_params_ptr_; }
  shared_ptr<RobotControlParameters> getCtrlParamsPtr() { return ctrl_params_ptr_; }
  shared_ptr<GamepadCommand> getGamepadCmdPtr() { return gamepad_cmd_ptr_; }
  virtual shared_ptr<spi_data_t> getSpiDataPtr() { return spi_data_ptr_; }
  virtual shared_ptr<spi_command_t> getSpiCommandPtr() { return spi_command_ptr_; }
  virtual shared_ptr<VectorNavData> getImuDataPtr() { return imu_data_ptr_; }
  PeriodicTaskManager* getTaskManagerPtr() { return &taskManager; }
  shared_ptr<CheaterState<double>> getCheaterStatePtr() { return cheater_state_ptr_; };

 protected:
  std::shared_ptr<RobotControlParameters> ctrl_params_ptr_;
  std::shared_ptr<LS_UserParameters> user_params_ptr_;
  std::shared_ptr<GamepadCommand> gamepad_cmd_ptr_;
  Gamepad gamepad_;
  argumentInfo info_;
  shared_ptr<spi_data_t> spi_data_ptr_;
  shared_ptr<spi_command_t> spi_command_ptr_;
  shared_ptr<VectorNavData> imu_data_ptr_;
  PeriodicTaskManager taskManager;
  shared_ptr<CheaterState<double>> cheater_state_ptr_;
  bool initialized_spiData_{false};
  bool initialized_spiCmd_{false};
  bool initialized_IMU_{false};

  // PrintTaskStatus statusTask;
};

#endif