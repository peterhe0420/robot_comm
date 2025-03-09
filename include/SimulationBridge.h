/*! @file SimulationBridge.h
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include <lcm-cpp.hpp>
#include <thread>

#include "BaseBridge.h"
#include "ControlParameters/RobotParameters.h"
#include "SimUtilities/SimulatorMessage.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"
#include "lcm-types/cpp/gamepad_lcmt.hpp"
#include "lcm-types/cpp/rc_to_lcm_t.hpp"
#include "rt/Gamepad.h"

class SimulationBridge : public BaseBridge {
 public:
  explicit SimulationBridge(const argumentInfo &info) {
    this->info_ = info;
    std::cout << "[SimulationBridge]robot type is:" << int(this->info_.robot_type) << std::endl;
  }
  // ~SimulationBridge() {
  //   // delete _fakeTaskManager;
  //   // delete _robotRunner;
  // }
  virtual void init(){};
  virtual void run();
  void handleControlParameters();
  void runRobotControl();
  // void run_sbus();
  // void runRemoteCtrlPadCallBack();
  // void runGamePadCallBack();
  // void testUDPRCLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const rc_to_lcm_t *msg);
  // void handleRCLCM();

 private:
  // void rcLcmThread() { while (true) { _rcsimLcm.handle(); } }
  // lcm::LCM _rcsimLcm;
  // std::thread _rcLcmThread;
  // PeriodicTaskManager taskManager;
  bool _firstControllerRun = true;
  SimulatorMode _simMode;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  // ControlParameters *user_params_ = nullptr;
  u64 _iterations = 0;
  std::thread *sbus_thread;
  std::thread *rc_thread;
};

#endif  // PROJECT_SIMULATIONDRIVER_H
