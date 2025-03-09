#ifndef A01A_HARDWARE_BRIDGE_H
#define A01A_HARDWARE_BRIDGE_H
#include "HardwareBridge.h"
#include "Types.h"
/*!
 * Interface between robot and hardware specialized for A01
 */
class A01aHardwareBridge : public HardwareBridge {
 public:
  A01aHardwareBridge(const argumentInfo &info);
  void logs_cleanup(const std::string &directoryPath, std::uintmax_t maxSizeThreshold, std::uintmax_t targetSize);
  virtual void init();
  void run();
  void initHardware();

  void runSpiCallBack();
  void runI2cCallBack();
  void runImuCallBack();

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
