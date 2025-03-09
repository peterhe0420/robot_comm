/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

// #include <lord_imu/LordImu.h>
// #include <sensor_msgs/Joy.h>

// #include <lcm-cpp.hpp>
// #include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <filesystem>
#include <lcm-cpp.hpp>
#include <lcm/lcm-cpp.hpp>

#include "BaseBridge.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/utilities.h"
#include "lcm-types/cpp/control_parameter_request_lcmt.hpp"
#include "lcm-types/cpp/control_parameter_respones_lcmt.hpp"
#include "lcm-types/cpp/gamepad_lcmt.hpp"
#include "lcm-types/cpp/microstrain_lcmt.hpp"
#include "lcm-types/cpp/spi_command_t.hpp"
#include "lcm-types/cpp/spi_data_t.hpp"
#include "rt/Gamepad.h"
#include "rt/batMgmt.h"
#include "rt/imu_service.h"
#include "rt/rt_i2c.h"
#include "rt/rt_leds.h"
#include "rt/rt_sbus.h"
#include "rt/rt_spi.h"
#include "rt/rt_udprc.h"

namespace fs = std::filesystem;

// uint16_t errcode = 0x000;
extern int leds_state;
extern spi_command_t spi_command_drv;
extern spi_data_t spi_data_drv;
#define MAXLOGSIZE 1024 * 1024 * 1024 * 2LL
#define HALFLOGSIZE 1024 * 1024 * 1024

/*!
 * Interface between robot and hardware
 */

extern batParamGet screen_info;
class HardwareBridge : public BaseBridge {
 public:
  HardwareBridge(const argumentInfo &info) : _interfaceLCM(getLcmUrl(255)) {
    this->info_ = info;
    auto i = getLcmUrl(255);
    // _interfaceLCM(getLcmUrl(255));
  }
  virtual void init() = 0;
  virtual void run() = 0;

  pthread_mutex_t infolog_mutex = PTHREAD_MUTEX_INITIALIZER;
  void infolog_run();
  int infolog_message();

  void prefaultStack();
  void setupScheduler();
  void initError(const char *reason, bool printErrno = false);
  void initCommon();
  void handleInterfaceLCM();
  void handleControlParameter(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const control_parameter_request_lcmt *msg);
  void initParameters();

 protected:
  void runinfologCallBack();
  PeriodicTaskManager taskManager;
  // PrintTaskStatus statusTask;
  // GamepadCommand _gamepadCommand;
  std::shared_ptr<gamepad_lcmt> gamepad_lcmt_msgptr_;

  lcm::LCM _interfaceLCM;
  control_parameter_respones_lcmt _parameter_response_lcmt;
  volatile bool _interfaceLcmQuit = false;
  std::thread *rc_thread;
  // Gamepad gamepad_;
  bool _firstRun = true;
  u64 _iterations = 0;
  std::thread _interfaceLcmThread;

  int _port;
};

#endif
