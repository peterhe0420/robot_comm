/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
// clang-format off
#include "HardwareBridge.h"

#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <thread>
#include <bitset>
#include "Configuration.h"
// #include "rt/rt_rc_interface.h"

// #include <rc_to_lcm_t.hpp>

#include "Utilities/Utilities_print.h"
#include"common/include/printLog.h"
#include <dirent.h>
#include <filesystem>

#include <iostream>
#include <vector>
#include <algorithm>
// clang-format on
uint16_t errcode = 0x000;
// #define MAXLOGSIZE  2 * 1024
// #define HALFLOGSIZE 1024
/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char* reason, bool printErrno) {
  LOG(ERROR) << "FAILED TO INITIALIZE HARDWARE: " << reason;
  if (printErrno) {
    LOG(ERROR) << "Error: " << strerror(errno);
  }
  exit(-1);
}

/*!
 * All hardware initialization steps that are common
 */
void HardwareBridge::initCommon() {
  LOG(INFO) << ("[HardwareBridge] Init stack\n");
  prefaultStack();
  LOG(INFO) << ("[HardwareBridge] Init scheduler\n");
  setupScheduler();
  if (!_interfaceLCM.good()) {
    initError("_interfaceLCM failed to initialize\n", false);
  }
  LOG(INFO) << ("[HardwareBridge] Subscribe LCM\n");
  _interfaceLCM.subscribe("interface_request", &HardwareBridge::handleControlParameter, this);
  LOG(INFO) << ("[HardwareBridge] Start interface LCM handler\n");
  _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
}

void HardwareBridge::run() {}
/*!
 * Run interface LCM
 */
void HardwareBridge::handleInterfaceLCM() {
  while (!_interfaceLcmQuit) _interfaceLCM.handle();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the A01 software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
void HardwareBridge::prefaultStack() {
  LOG(INFO) << ("[Init] Prefault stack...\n");
  volatile char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}

/*!
 * Configures the scheduler for real time priority
 */
void HardwareBridge::setupScheduler() {
  LOG(INFO) << ("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;
#if 1
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    // initError("sched_setscheduler failed.\n", true);
  }
#endif
}

void HardwareBridge::initParameters() {
  // load parameters from yaml files
  if (this->info_.load_from_file) {
    LOG(INFO) << ("[Hardware Bridge] Loading parameters from file...\n");
    try {
      if (this->info_.robot_type == RobotType::A01a) {
        this->ctrl_params_ptr_->initializeFromYamlFile(THIS_COM "config/A01a-defaults.yaml");
      } else {
        this->ctrl_params_ptr_->initializeFromYamlFile(THIS_COM "config/A01b-defaults.yaml");
      }
    } catch (std::exception& e) {
      LOG(ERROR) << "Failed to initialize robot parameters from yaml file: " << e.what();
      exit(1);
    }
    if (!this->ctrl_params_ptr_->isFullyInitialized()) {
      LOG(INFO) << ("Failed to initialize all robot parameters\n");
      exit(1);
    }
    LOG(INFO) << ("Loaded user parameters\n");
    if (this->user_params_ptr_.get()) {
      try {
        if (this->info_.robot_type == RobotType::A01a) {
          this->user_params_ptr_->initializeFromYamlFile(THIS_COM "config/A01a-ctrl-user-parameters.yaml");
        } else {
          this->user_params_ptr_->initializeFromYamlFile(THIS_COM "config/A01b-ctrl-user-parameters.yaml");
        }
      } catch (std::exception& e) {
        LOG(ERROR) << "Failed to initialize user parameters from yaml file: " << e.what();
        exit(1);
      }

      if (!this->user_params_ptr_->isFullyInitialized()) {
        LOG(INFO) << ("Failed to initialize all user parameters\n");
        exit(1);
      }

      LOG(INFO) << ("Loaded user parameters\n");
    } else {
      LOG(INFO) << ("Did not load user parameters because there aren't any\n");
    }
  } else {
    LOG(INFO) << ("[Hardware Bridge] Loading parameters over LCM...\n");
    while (!this->ctrl_params_ptr_->isFullyInitialized()) {
      LOG(INFO) << ("[Hardware Bridge] Waiting for robot parameters...\n");
      usleep(1000000);
    }

    if (this->user_params_ptr_.get()) {
      while (!this->user_params_ptr_->isFullyInitialized()) {
        LOG(INFO) << ("[Hardware Bridge] Waiting for user parameters...\n");
        usleep(1000000);
      }
    }
  }
  LOG(INFO) << ("[Hardware Bridge] Got all parameters, starting up!\n");
};

/*!
 * LCM Handler for control parameters
 */
void HardwareBridge::handleControlParameter(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_request_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) {
    // nothing to do!
    LOG(INFO) << (
        "[HardwareBridge] Warning: the interface has run a ControlParameter "
        "iteration, but there is no new request!\n");
    // return;
  }

  // sanity check
  s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
  if (nRequests != 1) {
    LOG(ERROR) << "[ERROR] Hardware bridge: we've missed " << nRequests - 1 << " requests";
  }

  switch (msg->requestKind) {
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      if (!this->user_params_ptr_.get()) {
        LOG(WARNING) << "[Warning] Got user param " << (char*)msg->name << ", but not using user parameters!";
      } else {
        std::string name((char*)msg->name);
        ControlParameter& param = this->user_params_ptr_->collection.lookup(name);

        // type check
        if ((s8)param._kind != msg->parameterKind) {
          throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                   " but received a command to set it to " +
                                   controlParameterValueKindToString((ControlParameterValueKind)msg->parameterKind));
        }
        // type check
        if ((s8)param._kind != msg->parameterKind) {
          throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                   " but received a command to set it to " +
                                   controlParameterValueKindToString((ControlParameterValueKind)msg->parameterKind));
        }

        // do the actual set
        this->user_params_ptr_->lockMutex();
        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);
        this->user_params_ptr_->unlockMutex();

        // respond:
        _parameter_response_lcmt.requestNumber = msg->requestNumber;  // acknowledge that the set has happened
        _parameter_response_lcmt.parameterKind = msg->parameterKind;  // just for debugging print statements
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
        // for debugging print statements
        strcpy((char*)_parameter_response_lcmt.name,
               name.c_str());  // just for debugging print statements
        _parameter_response_lcmt.requestKind = msg->requestKind;

        LOG(INFO) << "[User Control Parameter] set " << name.c_str() << " to " 
            << controlParameterValueToString(v, (ControlParameterValueKind)msg->parameterKind).c_str();
      }
    } break;

    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name((char*)msg->name);
      ControlParameter& param = this->ctrl_params_ptr_->collection.lookup(name);

      // type check
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                 " but received a command to set it to " +
                                 controlParameterValueKindToString((ControlParameterValueKind)msg->parameterKind));
      }
      // type check
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                 " but received a command to set it to " +
                                 controlParameterValueKindToString((ControlParameterValueKind)msg->parameterKind));
      }

      // do the actual set
      this->ctrl_params_ptr_->lockMutex();
      ControlParameterValue v;
      memcpy(&v, msg->value, sizeof(v));
      param.set(v, (ControlParameterValueKind)msg->parameterKind);
      this->ctrl_params_ptr_->unlockMutex();

      // respond:
      _parameter_response_lcmt.requestNumber = msg->requestNumber;  // acknowledge that the set has happened
      _parameter_response_lcmt.parameterKind = msg->parameterKind;  // just for debugging print statements
      memcpy(_parameter_response_lcmt.value, msg->value, 64);
      //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
      // for debugging print statements
      strcpy((char*)_parameter_response_lcmt.name,
             name.c_str());  // just for debugging print statements
      _parameter_response_lcmt.requestKind = msg->requestKind;

      LOG(INFO) << "[Robot Control Parameter] set " << name.c_str() << " to " 
          << controlParameterValueToString(v, (ControlParameterValueKind)msg->parameterKind).c_str();

    } break;

    default: {
      throw std::runtime_error("parameter type unsupported");
    } break;
  }
  _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}

void HardwareBridge::runinfologCallBack() { infolog_run(); }

int HardwareBridge::infolog_message() {
  // bat
  LOG(INFO) << "    **BAT:" << 1.0 * screen_info.cur / 100 << "A," << 1.0 * screen_info.vol / 1000 << "V,SOC:" << (uint16_t)screen_info.soc
            << "%,T-BAT:" << (int16_t)screen_info.temp << "℃,STATE:" << screen_info.state;
  // k temp
  LOG(INFO) << "**T0:" << screen_info.ktemp[0] << "℃, T1:" << screen_info.ktemp[1] << "℃, T2:" << screen_info.ktemp[2]
            << "℃, T3:" << screen_info.ktemp[3] << "℃";
  // cmd
  LOG(INFO) << "**CMD**q_des_abad[0]=" << spi_command_drv.q_des_abad[0] << ", [1]=" << spi_command_drv.q_des_abad[1]
            << ", [2]=" << spi_command_drv.q_des_abad[2] << ", [3]=" << spi_command_drv.q_des_abad[3];

  LOG(INFO) << "q_des_hip[0]=" << spi_command_drv.q_des_hip[0] << ", [1]=" << spi_command_drv.q_des_hip[1] << ", [2]=" << spi_command_drv.q_des_hip[2]
            << ", [3]=" << spi_command_drv.q_des_hip[3];

  LOG(INFO) << "q_des_knee[0]=" << spi_command_drv.q_des_knee[0] << ", [1]=" << spi_command_drv.q_des_knee[1]
            << ", [2]=" << spi_command_drv.q_des_knee[2] << ", [3]=" << spi_command_drv.q_des_knee[3];

  LOG(INFO) << "qd_des_abad[0]=" << spi_command_drv.qd_des_abad[0] << ", [1]=" << spi_command_drv.qd_des_abad[1]
            << ", [2]=" << spi_command_drv.qd_des_abad[2] << ", [3]=" << spi_command_drv.qd_des_abad[3];

  LOG(INFO) << "qd_des_hip[0]=" << spi_command_drv.qd_des_hip[0] << ", [1]=" << spi_command_drv.qd_des_hip[1]
            << ", [2]=" << spi_command_drv.qd_des_hip[2] << ", [3]=" << spi_command_drv.qd_des_hip[3];

  LOG(INFO) << "qd_des_knee[0]=" << spi_command_drv.qd_des_knee[0] << ", [1]=" << spi_command_drv.qd_des_knee[1]
            << ", [2]=" << spi_command_drv.qd_des_knee[2] << ", [3]=" << spi_command_drv.qd_des_knee[3];

  LOG(INFO) << "kp_abad[0]=" << spi_command_drv.kp_abad[0] << ", [1]=" << spi_command_drv.kp_abad[1] << ", [2]=" << spi_command_drv.kp_abad[2]
            << ", [3]=" << spi_command_drv.kp_abad[3];

  LOG(INFO) << "kp_hip[0]=" << spi_command_drv.kp_hip[0] << ", [1]=" << spi_command_drv.kp_hip[1] << ", [2]=" << spi_command_drv.kp_hip[2]
            << ", [3]=" << spi_command_drv.kp_hip[3];

  LOG(INFO) << "kp_knee[0]=" << spi_command_drv.kp_knee[0] << ", [1]=" << spi_command_drv.kp_knee[1] << ", [2]=" << spi_command_drv.kp_knee[2]
            << ", [3]=" << spi_command_drv.kp_knee[3];

  LOG(INFO) << "kd_abad[0]=" << spi_command_drv.kd_abad[0] << ", [1]=" << spi_command_drv.kd_abad[1] << ", [2]=" << spi_command_drv.kd_abad[2]
            << ", [3]=" << spi_command_drv.kd_abad[3];

  LOG(INFO) << "kd_hip[0]=" << spi_command_drv.kd_hip[0] << ", [1]=" << spi_command_drv.kd_hip[1] << ", [2]=" << spi_command_drv.kd_hip[2]
            << ", [3]=" << spi_command_drv.kd_hip[3];

  LOG(INFO) << "kd_knee[0]=" << spi_command_drv.kd_knee[0] << ", [1]=" << spi_command_drv.kd_knee[1] << ", [2]=" << spi_command_drv.kd_knee[2]
            << ", [3]=" << spi_command_drv.kd_knee[3];

  LOG(INFO) << "tau_abad_ff[0]=" << spi_command_drv.tau_abad_ff[0] << ", [1]=" << spi_command_drv.tau_abad_ff[1]
            << ", [2]=" << spi_command_drv.tau_abad_ff[2] << ", [3]=" << spi_command_drv.tau_abad_ff[3];

  LOG(INFO) << "tau_hip_ff[0]=" << spi_command_drv.tau_hip_ff[0] << ", [1]=" << spi_command_drv.tau_hip_ff[1]
            << ", [2]=" << spi_command_drv.tau_hip_ff[2] << ", [3]=" << spi_command_drv.tau_hip_ff[3];

  LOG(INFO) << "tau_knee_ff[0]=" << spi_command_drv.tau_knee_ff[0] << ", [1]=" << spi_command_drv.tau_knee_ff[1]
            << ", [2]=" << spi_command_drv.tau_knee_ff[2] << ", [3]=" << spi_command_drv.tau_knee_ff[3];

  LOG(INFO) << "flags[0]=" << spi_command_drv.flags[0] << ", [1]=" << spi_command_drv.flags[1] << ", [2]=" << spi_command_drv.flags[2]
            << ", [3]=" << spi_command_drv.flags[3];
  // data
  LOG(INFO) << "**DATA**q_abad[0]=" << spi_data_drv.q_abad[0] << ", [1]=" << spi_data_drv.q_abad[1] << ", [2]=" << spi_data_drv.q_abad[2]
            << ", [3]=" << spi_data_drv.q_abad[3];
  LOG(INFO) << "q_hip[0]=" << spi_data_drv.q_hip[0] << ", [1]=" << spi_data_drv.q_hip[1] << ", [2]=" << spi_data_drv.q_hip[2]
            << ", [3]=" << spi_data_drv.q_hip[3];
  LOG(INFO) << "q_knee[0]=" << spi_data_drv.q_knee[0] << ", [1]=" << spi_data_drv.q_knee[1] << ", [2]=" << spi_data_drv.q_knee[2]
            << ", [3]=" << spi_data_drv.q_knee[3];
  LOG(INFO) << "qd_abad[0]=" << spi_data_drv.qd_abad[0] << ", [1]=" << spi_data_drv.qd_abad[1] << ", [2]=" << spi_data_drv.qd_abad[2]
            << ", [3]=" << spi_data_drv.qd_abad[3];
  LOG(INFO) << "qd_hip[0]=" << spi_data_drv.qd_hip[0] << ", [1]=" << spi_data_drv.qd_hip[1] << ", [2]=" << spi_data_drv.qd_hip[2]
            << ", [3]=" << spi_data_drv.qd_hip[3];
  LOG(INFO) << "qd_knee[0]=" << spi_data_drv.qd_knee[0] << ", [1]=" << spi_data_drv.qd_knee[1] << ", [2]=" << spi_data_drv.qd_knee[2]
            << ", [3]=" << spi_data_drv.qd_knee[3];
  LOG(INFO) << "tau_abad[0]=" << spi_data_drv.tau_abad[0] << ", [1]=" << spi_data_drv.tau_abad[1] << ", [2]=" << spi_data_drv.tau_abad[2]
            << ", [3]=" << spi_data_drv.tau_abad[3];
  LOG(INFO) << "tau_hip[0]=" << spi_data_drv.tau_hip[0] << ", [1]=" << spi_data_drv.tau_hip[1] << ", [2]=" << spi_data_drv.tau_hip[2]
            << ", [3]=" << spi_data_drv.tau_hip[3];
  LOG(INFO) << "tau_knee[0]=" << spi_data_drv.tau_knee[0] << ", [1]=" << spi_data_drv.tau_knee[1] << ", [2]=" << spi_data_drv.tau_knee[2]
            << ", [3]=" << spi_data_drv.tau_knee[3];
  LOG(INFO) << "flags[0]=" << spi_data_drv.flags[0] << ", [1]=" << spi_data_drv.flags[1] << ", [2]=" << spi_data_drv.flags[2]
            << ", [3]=" << spi_data_drv.flags[3];

  return 0;
};

void HardwareBridge::infolog_run() {
  pthread_mutex_lock(&infolog_mutex);
  infolog_message();
  pthread_mutex_unlock(&infolog_mutex);
}
