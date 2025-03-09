/*! @file SimulationBridge.cpp
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */
// clang-format off
// #include "StateEstimator/LegController.h"
#include "SimulationBridge.h"
#include "lcm-types/cpp/rc_to_lcm_t.hpp"
#include "Utilities/SegfaultHandler.h"
#include "printLog.h"
#include "rt/rt_sbus.h"
#include "rt/rt_udprc.h"
#include <glog/logging.h>
// clang-format on
extern int32_t update_rc;

/*!
 * Connect to a simulation
 */
void SimulationBridge::run() {
  // init shared memory:
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();

  install_segfault_handler(_sharedMemory().robotToSim.errorMessage);

  try {
    LOG(INFO) << ("[Simulation Driver] Starting main loop...\n");
    bool firstRun = true;
    for (;;) {
      // wait for our turn to access the shared memory
      // on the first loop, this gives the simulator a chance to put stuff in
      // shared memory before we start
      _sharedMemory().waitForSimulator();

      if (firstRun) {
        firstRun = false;

        // check that the robot type is correct:
        if (this->info_.robot_type != _sharedMemory().simToRobot.robotType) {
          LOG(ERROR) << "simulator and simulatorDriver don't agree on which robot we are simulating (robot: "
              << (int)this->info_.robot_type << ", sim: " << (int)_sharedMemory().simToRobot.robotType;
          throw std::runtime_error("robot mismatch!");
        }
      }

      // the simulator tells us which mode to run in
      _simMode = _sharedMemory().simToRobot.mode;

      switch (_simMode) {
        case SimulatorMode::RUN_CONTROL_PARAMETERS:  // there is a new control
          // parameter request
          handleControlParameters();
          break;
        case SimulatorMode::RUN_CONTROLLER:  // the simulator is ready for the
          // next robot controller run
          _iterations++;
          runRobotControl();
          break;
        case SimulatorMode::DO_NOTHING:  // the simulator is just checking to see
          // if we are alive yet
          break;
        case SimulatorMode::EXIT:  // the simulator is done with us
          LOG(INFO) << ("[Simulation Driver] Transitioned to exit mode\n");
          return;
          break;
        default:
          throw std::runtime_error("unknown simulator mode");
      }

      // tell the simulator we are done
      _sharedMemory().robotIsDone();
    }
  } catch (std::exception &e) {
    strncpy(_sharedMemory().robotToSim.errorMessage, e.what(), sizeof(_sharedMemory().robotToSim.errorMessage));
    _sharedMemory().robotToSim.errorMessage[sizeof(_sharedMemory().robotToSim.errorMessage) - 1] = '\0';
    throw e;
  }
}

/*!
 * This function handles a a control parameter message from the simulator
 */
void SimulationBridge::handleControlParameters() {
  ControlParameterRequest &request = _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse &response = _sharedMemory().robotToSim.controlParameterResponse;
  if (request.requestNumber <= response.requestNumber) {
    // nothing to do!
    LOG(INFO) << "[SimulationBridge] Warning: the simulator has run a ControlParameter "
         << "iteration, but there is no new request!";
    return;
  }

  // sanity check
  u64 nRequests = request.requestNumber - response.requestNumber;
  assert(nRequests == 1);

  response.nParameters = this->ctrl_params_ptr_->collection._map.size();  // todo don't do this every single time?

  LOG(INFO) << "request kind: " << (int)request.requestKind << ", name: " << request.name;
  switch (request.requestKind) {
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter &param = this->ctrl_params_ptr_->collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                 " but received a command to set it to " + controlParameterValueKindToString(request.parameterKind));
      }

      // do the actual set
      this->ctrl_params_ptr_->lockMutex();
      param.set(request.value, request.parameterKind);
      this->ctrl_params_ptr_->unlockMutex();

      // respond:
      response.requestNumber = request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind = request.parameterKind;  // just for debugging print statements
      response.value = request.value;                  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      LOG(INFO) << response.toString().c_str();

    } break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      std::string name(request.name);
      if (!this->user_params_ptr_) {
        LOG(INFO) << "[Simulation Bridge] Warning: tried to set user parameter, but "
            "the robot does not have any!";
      } else {
        ControlParameter &param = this->user_params_ptr_->collection.lookup(name);

        // type check
        if (param._kind != request.parameterKind) {
          throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                   " but received a command to set it to " + controlParameterValueKindToString(request.parameterKind));
        }

        // do the actual set
        this->user_params_ptr_->lockMutex();
        param.set(request.value, request.parameterKind);
        this->user_params_ptr_->unlockMutex();
      }

      // respond:
      response.requestNumber = request.requestNumber;  // acknowledge that the set has happened
      response.parameterKind = request.parameterKind;  // just for debugging print statements
      response.value = request.value;                  // just for debugging print statements
      strcpy(response.name,
             name.c_str());  // just for debugging print statements
      response.requestKind = request.requestKind;

      LOG(INFO) << response.toString().c_str();

    } break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME: {
      std::string name(request.name);
      ControlParameter &param = this->ctrl_params_ptr_->collection.lookup(name);

      // type check
      if (param._kind != request.parameterKind) {
        throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is " + controlParameterValueKindToString(param._kind) +
                                 " but received a command to set it to " + controlParameterValueKindToString(request.parameterKind));
      }

      // respond
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;  // acknowledge
      response.parameterKind = request.parameterKind;  // just for debugging print statements
      strcpy(response.name,
             name.c_str());                        // just for debugging print statements
      response.requestKind = request.requestKind;  // just for debugging print statements

      LOG(INFO) << response.toString().c_str();
    } break;
    default:
      throw std::runtime_error("unhandled get/set");
  }
}

/*!
 * Run the robot controller
 */
void SimulationBridge::runRobotControl() {
  if (_firstControllerRun) {
    LOG(INFO) << ("[Simulator Driver] First run of robot controller...\n");
    if (this->ctrl_params_ptr_->isFullyInitialized()) {
      LOG(INFO) << "\tAll " << this->ctrl_params_ptr_->collection._map.size() << " control parameters are initialized";
    } else {
      LOG(ERROR) << "\tbut not all control parameters were initialized. Missing: \n" 
          << this->ctrl_params_ptr_->generateUnitializedList().c_str();
      throw std::runtime_error("not all parameters initialized when going into RUN_CONTROLLER");
    }

    // auto *userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();
    if (this->user_params_ptr_) {
      if (this->user_params_ptr_->isFullyInitialized()) {
        LOG(INFO) << "\tAll " << this->user_params_ptr_->collection._map.size() << " user parameters are initialized";
        _simMode = SimulatorMode::RUN_CONTROLLER;
      } else {
        LOG(ERROR) << "\tbut not all control parameters were initialized. Missing:\n" 
              << this->user_params_ptr_->generateUnitializedList().c_str();
        throw std::runtime_error("not all parameters initialized when going into RUN_CONTROLLER");
      }
    } else {
      _simMode = SimulatorMode::RUN_CONTROLLER;
    }

    spi_data_ptr_.reset(&_sharedMemory().simToRobot.spiData);
    imu_data_ptr_.reset(&_sharedMemory().simToRobot.vectorNav);
    spi_command_ptr_.reset(&_sharedMemory().robotToSim.spiCommand);
    cheater_state_ptr_.reset(&_sharedMemory().simToRobot.cheaterState);
    _firstControllerRun = false;
    initialized_IMU_ = true;
    initialized_spiData_ = true;
    initialized_spiCmd_ = true;
    if (this->ctrl_params_ptr_->use_gamepad < 0.5) {
      init_udprc();
      rc_thread = new std::thread(&SimulationBridge::runRemoteCtrlPadCallBack, this);
    } else {
      this->gamepad_.init();
      rc_thread = new std::thread(&SimulationBridge::runGamePadCallBack, this);
    }
  }
}
// tes
