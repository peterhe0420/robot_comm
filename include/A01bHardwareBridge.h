#ifndef A01B_HARDWARE_BRIDGE_H
#define A01B_HARDWARE_BRIDGE_H
#include "HardwareBridge.h"
/*!
 * Interface between robot and hardware specialized for A01
 */
class A01bHardwareBridge : public HardwareBridge {
 public:
  A01bHardwareBridge(const argumentInfo &info);
  void logs_cleanup(const std::string &directoryPath, std::uintmax_t maxSizeThreshold, std::uintmax_t targetSize);
  void run();
  void initHardware();
  virtual void init();

  void runSpiCallBack();
  void runI2cCallBack();
  void runImuCallBack();
  void runRemoteCtrlPadCallBack();

 private:
  // imu data
  // VectorNavData _vectorNavData;

  std::thread _microstrainThread;
  // LordImu _microstrainImu;
  IMUService imuServ;
  batMgmt bat;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
};
#endif
