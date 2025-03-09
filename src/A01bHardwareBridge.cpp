// clang-format off
#include "A01bHardwareBridge.h"
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <thread>
#include <bitset>
#include "Configuration.h"
// #include "rt/rt_rc_interface.h"

#include "lcm-types/cpp/rc_to_lcm_t.hpp"
#include "Utilities/Utilities_print.h"
#include"common/include/printLog.h"
#include <dirent.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdlib> 
// clang-format on
A01bHardwareBridge::A01bHardwareBridge(const argumentInfo& info) : HardwareBridge(info) {  //_spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255))
}

void A01bHardwareBridge::logs_cleanup(const std::string& directoryPath, std::uintmax_t maxSizeThreshold, std::uintmax_t targetSize) {
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
      totalSize -= fs::file_size(files[filesToDelete]);
      fs::remove(files[filesToDelete]);
      ++filesToDelete;
    }

    // std::cout << filesToDelete << " oldest files deleted.\n";
  } else {
    // std::cout << "No files deleted.\n";
  }
}

void A01bHardwareBridge::init() {
  initCommon();
  initHardware();
  initParameters();
}
/*!
 * Main method for A01 hardware
 */
void A01bHardwareBridge::run() {

  init();
  // init control thread
  // statusTask.start();

  /*spiTask start 2ms
   */
  PeriodicMemberFunction<A01bHardwareBridge> spiTask(&taskManager, 0.002, "spi", &A01bHardwareBridge::runSpiCallBack, this);
  spiTask.start();

  /*infologTask start 100ms
   */
  PeriodicMemberFunction<A01bHardwareBridge> infologTask(&taskManager, 0.1, "infolog", &A01bHardwareBridge::runinfologCallBack, this);
  infologTask.start();

  /*start gamepad or remote control pad
   *rc + lidar(ros)
   */
  // if (_robotParams.use_gamepad < 0.5) {
  // std::cout << "run remote gamepad" << std::endl;
  init_udprc();
  rc_thread = new std::thread(&BaseBridge::runRemoteCtrlPadCallBack, this);
  // } else {
  gamepad_.init();
  rc_thread = new std::thread(&HardwareBridge::runGamePadCallBack, this);
  // }

  /*start imu microstrain */
  // if (_microstrainInit) {
  //   PeriodicMemberFunction<A01bHardwareBridge> imuTask(&taskManager, 0.003, "imu", &A01bHardwareBridge::runImuCallBack, this);
  //   imuTask.start();
  // }
  if (_microstrainInit) _microstrainThread = std::thread(&A01bHardwareBridge::runImuCallBack, this);

  /*set datas pointer to robot runner */

  _firstRun = false;
  // green LED
  if (leds_state == 0) {
    leds_state = 1;
    LOG(INFO) << ("[Hardware Bridge] GREEN LED ON! NORMAL\n");
  } else {
    LOG(INFO) << ("[Hardware Bridge] GREEN LED OFF! ABNORMAL\n");
  }
  for (;;) {
    if (screen_info.soc < 5 && leds_state != 6 && leds_state != 2 && leds_state != 4) {
      LOG(INFO) << "[Hardware Bridge] RED YELLOW LED ON! SOC = " << screen_info.soc << " <5";
      leds_state = 6;
    } else if (screen_info.soc >= 5 && screen_info.soc < 20 && leds_state != 3 && leds_state != 2 && leds_state != 4) {
      LOG(INFO) << "[Hardware Bridge] GREEN YELLOW LED ON! SOC = " << screen_info.soc << " <20";
      leds_state = 3;
    } else if (screen_info.soc >= 20 && leds_state != 1 && leds_state != 2 && leds_state != 4) {
      leds_state = 1;
      LOG(INFO) << "[Hardware Bridge] GREEN LED ON! SOC = " << screen_info.soc << " >=20";
    }
    rt_leds_state();
    usleep(1000000);
  }
  google::ShutdownGoogleLogging();  //当要结束glog时必须关闭库，否则会内存溢出
}

void A01bHardwareBridge::runImuCallBack() {
  IMU_DATA data;
  while (true) {
    imuServ.procOnce(IMU_YIS300_DK);
    data = imuServ.getData();
    this->imu_data_ptr_->accelerometer << data.ax, data.ay, data.az;
    this->imu_data_ptr_->gyro << data.vx, data.vy, data.vz;
    this->imu_data_ptr_->quat << data.qw, data.qx, data.qy, data.qz;
    usleep(2000);
    initialized_IMU_ = true;
  }
}

static int bat_callback(batParamGet param, void* contex) {
  memcpy((void*)&screen_info, (void*)&param, sizeof(batParamGet));
  return 0;
}

/*!
 * Initialize   A01 specific hardware
 */
void A01bHardwareBridge::initHardware() {
  bat.initCanIDb(0x7F2);  // bat
  bat.initConn();
  bat.regCallback(bat_callback, NULL);
  std::uintmax_t maxSizeThreshold = MAXLOGSIZE;
  std::uintmax_t targetSize = HALFLOGSIZE;

  logs_cleanup("./logs", maxSizeThreshold, targetSize);

  while (screen_info.cur > 0) {
    usleep(1000000);
    // system("sudo shutdown -h now");
  }
  usleep(1000000);

  init_spi();
  // _microstrainInit = _microstrainImu.tryInit(0, 921600);
  _microstrainInit = imuServ.init();
  rt_leds_init();
}

/*!
 * Run   A01 SPI
 */
void A01bHardwareBridge::runSpiCallBack() {
  spi_command_t* cmd = get_spi_command();
  spi_data_t* data = get_spi_data();
  memcpy(cmd, this->spi_command_ptr_.get(), sizeof(spi_command_t));
  spi_driver_run();
  memcpy(this->spi_data_ptr_.get(), data, sizeof(spi_data_t));
  initialized_spiData_ = true;
  initialized_spiCmd_ = true;
}
