// clang-format off
#include "A01aHardwareBridge.h"
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <thread>
#include <bitset>
#include "Configuration.h"
// #include "rt/rt_rc_interface.h"

#include <lcm-types/cpp/rc_to_lcm_t.hpp>

#include "Utilities/Utilities_print.h"
#include"common/include/printLog.h"
#include <dirent.h>
// #include <filesystem>

#include <iostream>
#include <vector>
#include <algorithm>
// clang-format on
A01aHardwareBridge::A01aHardwareBridge(const argumentInfo& info) : HardwareBridge(info) {  //_spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255))
}

void A01aHardwareBridge::logs_cleanup(const std::string& directoryPath, std::uintmax_t maxSizeThreshold, std::uintmax_t targetSize) {
  std::vector<fs::path> files;
  std::uintmax_t totalSize = 0;

  // 遍历目录，获取文件信息并计算总大小
  for (const auto& entry : fs::directory_iterator(directoryPath)) {
    if (fs::is_regular_file(entry)) {
      files.push_back(entry.path());
      totalSize += fs::file_size(entry.path());
    }
  }

  // std::cout << "Total size: " << totalSize / (1024 * 1024) << " MB\n";
  // std::cout << "Total size: " << totalSize / 1024 << " kB\n";//for test

  // 如果总大小超过 maxSizeThreshold，计算删除文件需要释放的空间
  if (totalSize > maxSizeThreshold && !files.empty()) {
    // 按文件修改时间排序
    std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) { return fs::last_write_time(a) < fs::last_write_time(b); });

    // 删除文件直到总大小降至 targetSize 以下
    size_t filesToDelete = 0;
    while (totalSize > targetSize && filesToDelete < files.size()) {
      totalSize -= fs::file_size(files[filesToDelete]) / (1024 * 1024);
      fs::remove(files[filesToDelete]);
      ++filesToDelete;
    }

    // std::cout << filesToDelete << " oldest files deleted.\n";
  } else {
    // std::cout << "No files deleted.\n";
  }
}

// int test() {
// std::string directoryPath = "your_directory_path_here";
// std::uintmax_t maxSizeThreshold = 2 * 1024 * 1024 * 1024;  // 2GB
// std::uintmax_t targetSize = 1 * 1024 * 1024 * 1024;        // 1GB

// 调用封装的功能函数
// logs_cleanup(directoryPath, maxSizeThreshold, targetSize);

// return 0;
// }
void A01aHardwareBridge::init() {
  initCommon();
  initHardware();
  initParameters();
}
/*!
 * Main method for A01 hardware
 */
void A01aHardwareBridge::run() {

  this->init();

  /*spiTask start 2ms
   */
  PeriodicMemberFunction<A01aHardwareBridge> spiTask(&taskManager, 0.002, "spi", &A01aHardwareBridge::runSpiCallBack, this);
  spiTask.start();

  /*i2cTask start 3s
   */
  PeriodicMemberFunction<A01aHardwareBridge> i2cTask(&taskManager, 3, "i2c", &A01aHardwareBridge::runI2cCallBack, this);
  i2cTask.start();
  /*infologTask start 100ms
   */
  PeriodicMemberFunction<A01aHardwareBridge> infologTask(&taskManager, 0.1, "infolog", &A01aHardwareBridge::runinfologCallBack, this);
  infologTask.start();

  // if (_robotParams.use_gamepad < 0.5) {
  // std::cout << "run remote gamepad" << std::endl;
  init_udprc();
  rc_thread = new std::thread(&BaseBridge::runRemoteCtrlPadCallBack, this);
  // } else {
  gamepad_.init();
  rc_thread = new std::thread(&BaseBridge::runGamePadCallBack, this);
  // }

  /*start imu microstrain */
  if (_microstrainInit) {
    PeriodicMemberFunction<A01aHardwareBridge> imuTask(&taskManager, 0.0025, "imu", &A01aHardwareBridge::runImuCallBack, this);
    imuTask.start();
  }
  // if (_microstrainInit) _microstrainThread = std::thread(&A01aHardwareBridge::runImuCallBack, this);

  /*set datas pointer to robot runner */
  _firstRun = false;
  for (;;) {
    usleep(1000000);
  }
  google::ShutdownGoogleLogging();  //当要结束glog时必须关闭库，否则会内存溢出
}

void A01aHardwareBridge::runImuCallBack() {
  IMU_DATA data;
  while (true) {
    imuServ.procOnce(IMU_YIS300_DK);
    data = imuServ.getData();
    this->imu_data_ptr_->accelerometer << data.ax, data.ay, data.az;
    this->imu_data_ptr_->gyro << data.vx, data.vy, data.vz;
    this->imu_data_ptr_->quat << data.qw, data.qx, data.qy, data.qz;
    usleep(200);
    initialized_IMU_ = true;
  }
}

static int bat_callback(batParamGet param, void* contex) {
  memcpy((void*)&screen_info, (void*)&param, sizeof(batParamGet));
  // LDEBUG(LDEBUG_NOTICE, "\nCUR: %dmA, VOL: %dmV, SOC: %d, T-BAT: %d℃, STATE: %X\n", param.cur * 10, param.vol, param.soc, param.temp, param.state);
  // LDEBUG(LDEBUG_NOTICE, "T0: %.1f℃, T1: %.1f℃, T2: %.1f℃, T3: %.1f℃\n", param.ktemp[0], param.ktemp[1], param.ktemp[2], param.ktemp[3]);
  return 0;
}

/*!
 * Initialize   A01 specific hardware
 */
void A01aHardwareBridge::initHardware() {
  bat.initCanID(0x7F2, 0x301);  // bat pt100
  bat.initConn();
  bat.regCallback(bat_callback, NULL);
  screen_init();
  // usleep(2000000);

  std::uintmax_t maxSizeThreshold = MAXLOGSIZE;
  std::uintmax_t targetSize = HALFLOGSIZE;

  logs_cleanup("./logs", maxSizeThreshold, targetSize);

  // LOG(ERROR) << "ERROR TEST";

  init_spi();
  // spinor
  // _microstrainInit = _microstrainImu.tryInit(0, 921600);
  _microstrainInit = imuServ.init();
}

/*!
 * Run   A01 SPI
 */
void A01aHardwareBridge::runSpiCallBack() {
  spi_command_t* cmd = get_spi_command();
  spi_data_t* data = get_spi_data();
  memcpy(cmd, this->spi_command_ptr_.get(), sizeof(spi_command_t));
  spi_driver_run();
  memcpy(this->spi_data_ptr_.get(), data, sizeof(spi_data_t));
  initialized_spiData_ = true;
  initialized_spiCmd_ = true;
}

/*!
 * Run   A01 I2C
 */
void A01aHardwareBridge::runI2cCallBack() { i2c_driver_run(); }
