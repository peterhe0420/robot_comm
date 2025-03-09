/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */

#include <JetsonGPIO.h>
#include <byteswap.h>
#include <glog/logging.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/timeb.h>
#include <sys/types.h>
#include <time.h>

#include <bitset>
#include <lcm/lcm-cpp.hpp>

#include "common/include/printLog.h"
#include "ros_array_interface.h"
#include "rt/CanProt.h"
#include "rt/batMgmt.h"
#include "rt/rt_spi.h"

// #define DEBUG_SPI

#define SPI_9M 9000000
// #define SPI_12M 12000000
uint16_t errcode_old = 0x000;
unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 16;
unsigned int spi_speed = SPI_9M;
uint8_t lsb = 0x01;

int spi_1_fd = -1;
int spi_2_fd = -1;

int spi_open();
extern int leds_state;
extern uint16_t errcode;
batParamGet screen_info;  // extern
static spine_cmd_t g_spine_cmd;
static spine_data_t g_spine_data, g_spine_data_old[2];

uint8_t temperatureget = 0;
spi_command_t spi_command_drv;
spi_data_t spi_data_drv;
spi_torque_t spi_torque;

pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER;
static MotorMgmt motorMgmt;
const float max_torque[3] = {17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

// only used for actual robot
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};

// only used for actual robot
const float abad_offset[4] = {-K_ABAD_OFFSET_POS, K_ABAD_OFFSET_POS, -K_ABAD_OFFSET_POS, K_ABAD_OFFSET_POS};
const float hip_offset[4] = {0.f, 0.f, 0.f, 0.f};  //{M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
const float knee_offset[4] = {K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};

int warning_level_max = 0;
double temperature_group_max = 0;
/*!
 * Compute SPI message checksum
 * @param data : input
 * @param len : length (in 32-bit words)
 * @return
 */
uint32_t xor_checksum(uint32_t *data, size_t len) {
  uint32_t t = 0;
  for (size_t i = 0; i < len; i++) t = t ^ data[i];
  return t;
}

/*!
 * Emulate the spi board to estimate the torque.
 */
void fake_spine_control(spi_command_t *cmd, spi_data_t *data, spi_torque_t *torque_out, int board_num) {
  torque_out->tau_abad[board_num] = cmd->kp_abad[board_num] * (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
                                    cmd->kd_abad[board_num] * (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) + cmd->tau_abad_ff[board_num];

  torque_out->tau_hip[board_num] = cmd->kp_hip[board_num] * (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
                                   cmd->kd_hip[board_num] * (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) + cmd->tau_hip_ff[board_num];

  torque_out->tau_knee[board_num] = cmd->kp_knee[board_num] * (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
                                    cmd->kd_knee[board_num] * (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) + cmd->tau_knee_ff[board_num];

  const float *torque_limits = disabled_torque;

  if (cmd->flags[board_num] & 0b1) {
    if (cmd->flags[board_num] & 0b10)
      torque_limits = wimp_torque;
    else
      torque_limits = max_torque;
  }

  if (torque_out->tau_abad[board_num] > torque_limits[0]) torque_out->tau_abad[board_num] = torque_limits[0];
  if (torque_out->tau_abad[board_num] < -torque_limits[0]) torque_out->tau_abad[board_num] = -torque_limits[0];

  if (torque_out->tau_hip[board_num] > torque_limits[1]) torque_out->tau_hip[board_num] = torque_limits[1];
  if (torque_out->tau_hip[board_num] < -torque_limits[1]) torque_out->tau_hip[board_num] = -torque_limits[1];

  if (torque_out->tau_knee[board_num] > torque_limits[2]) torque_out->tau_knee[board_num] = torque_limits[2];
  if (torque_out->tau_knee[board_num] < -torque_limits[2]) torque_out->tau_knee[board_num] = -torque_limits[2];
}

/*!
 * Initialize SPI
 */
void init_spi() {
  // check sizes:
  size_t command_size = sizeof(spi_command_t);
  size_t data_size = sizeof(spi_data_t);

  memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  memset(&spi_data_drv, 0, sizeof(spi_data_drv));

  if (pthread_mutex_init(&spi_mutex, NULL) != 0) LOG_INFO("[ERROR: RT SPI] Failed to create spi data mutex\n");

  if (command_size != K_EXPECTED_COMMAND_SIZE) {
    LOG_INFO("[RT SPI] Error command size is %ld, expected %d\n", command_size, K_EXPECTED_COMMAND_SIZE);
  } else
    LOG_INFO("[RT SPI] command size good\n");

  if (data_size != K_EXPECTED_DATA_SIZE) {
    LOG_INFO("[RT SPI] Error data size is %ld, expected %d\n", data_size, K_EXPECTED_DATA_SIZE);
  } else
    LOG_INFO("[RT SPI] data size good\n");

  spi_open();
}

/*!
 * Open SPI device
 */

int spi_open() {
  int rv = 0;
  spi_1_fd = open("/dev/spidev0.0", O_RDWR);
  if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 0.0");
  spi_2_fd = open("/dev/spidev0.1", O_RDWR);
  if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 0.1");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");

  motorMgmt.initMotor();

  usleep(5000000);

  return rv;
}

int spi_driver_iterations = 0;
static char rec2_1 = 0;
/*!
 * convert spi command to spine_cmd_t
 */
void spi_to_spine(spi_command_t *cmd, spine_cmd_t *spine_cmd, int stm32_num) {
  for (int i = 0; i < 2; i++) {
    spine_cmd->q_des_abad[i] = (cmd->q_des_abad[i + stm32_num] - abad_offset[i + stm32_num]) * abad_side_sign[i + stm32_num];
    spine_cmd->q_des_hip[i] = (cmd->q_des_hip[i + stm32_num] - hip_offset[i + stm32_num]) * hip_side_sign[i + stm32_num];
    spine_cmd->q_des_knee[i] = (cmd->q_des_knee[i + stm32_num] - knee_offset[i + stm32_num]) * knee_side_sign[i + stm32_num];

    spine_cmd->qd_des_abad[i] = cmd->qd_des_abad[i + stm32_num] * abad_side_sign[i + stm32_num];
    spine_cmd->qd_des_hip[i] = cmd->qd_des_hip[i + stm32_num] * hip_side_sign[i + stm32_num];
    spine_cmd->qd_des_knee[i] = cmd->qd_des_knee[i + stm32_num] * knee_side_sign[i + stm32_num];

    spine_cmd->kp_abad[i] = cmd->kp_abad[i + stm32_num];
    spine_cmd->kp_hip[i] = cmd->kp_hip[i + stm32_num];
    spine_cmd->kp_knee[i] = cmd->kp_knee[i + stm32_num];

    spine_cmd->kd_abad[i] = cmd->kd_abad[i + stm32_num];
    spine_cmd->kd_hip[i] = cmd->kd_hip[i + stm32_num];
    spine_cmd->kd_knee[i] = cmd->kd_knee[i + stm32_num];

    spine_cmd->tau_abad_ff[i] = cmd->tau_abad_ff[i + stm32_num] * abad_side_sign[i + stm32_num];
    spine_cmd->tau_hip_ff[i] = cmd->tau_hip_ff[i + stm32_num] * hip_side_sign[i + stm32_num];
    spine_cmd->tau_knee_ff[i] = cmd->tau_knee_ff[i + stm32_num] * knee_side_sign[i + stm32_num];

    spine_cmd->flags[i] = cmd->flags[i + stm32_num];  // spinor
    // LOG_INFO("spine_cmd->tau_knee_ff leg=[%d], tau_knee=[%f]", i + stm32_num, spine_cmd->tau_knee_ff[i]);
    // LOG_INFO("spine_cmd->leg=[%d], q_des_knee=[%f], qd_des_knee=[%f], kp_knee=[%f], kd_knee=[%f], tau_knee_ff=[%f]\n", i + stm32_num,
    //          spine_cmd->q_des_knee[i + stm32_num], spine_cmd->qd_des_knee[i + stm32_num], spine_cmd->kp_knee[i + stm32_num],
    //          spine_cmd->kd_knee[i + stm32_num], spine_cmd->tau_knee_ff[i + stm32_num]);
    if (temperatureget) {
      spine_cmd->flags[i] = 0xC0;
    }
  }
  spine_cmd->checksum = xor_checksum((uint32_t *)spine_cmd + 1, 32);
  // LOG_INFO("[RT SPI] spine_cmd[%d] checksum=  0x%08x\n", stm32_num/2,spine_cmd->checksum);
}

/*!
 * convert spine_data_t to spi data
 */
struct timeb timebufferspi[2];
void spine_to_spi(spi_data_t *data, spine_data_t *spine_data, int stm32_num) {
  static spi_data_t data_old;
  char *timeline;

#ifdef DEBUG_SPI
  static RosArrayInterface rospub;
  static bool init_ros{false};
  if (!init_ros) {
    rospub.init("actuator_temperature");
    init_ros = true;
  }
  static float motor_temperature[12];
  static float driver_temperature[12];
  static float BMS_msg[6];
  static int motor_warning_level, soc_warning_level;
#endif

  uint32_t calc_checksum = xor_checksum((uint32_t *)spine_data, 20);
  if (calc_checksum != (uint32_t)spine_data->checksum) {
    LOG_INFO("SPI ERROR BAD CHECKSUM GOT 0x%08x EXPECTED 0x%08x\n", calc_checksum, (uint32_t)spine_data->checksum);
    memcpy(spine_data, &g_spine_data_old[stm32_num / 2], 20 * 4);
    spine_data->flags[0] = 0x0A;  // spi crc error code 0x0A
    spine_data->flags[1] = 0x0A;
  }
  memcpy(&g_spine_data_old[stm32_num / 2], spine_data, 20 * 4);

  for (int i = 0; i < 2; i++) {
    data->flags[i + stm32_num] = spine_data->flags[i];
    if (data->flags[i + stm32_num] == 0xC000) {
      //温度反馈
      LOG(INFO) << "    **Motor -" << i + stm32_num << "-TAHK:" << spine_data->qd_abad[i] << "℃," << spine_data->qd_hip[i] << "℃,"
                << spine_data->qd_knee[i] << "℃";
      LOG(INFO) << "    **Driver-" << i + stm32_num << "-TAHK:" << spine_data->tau_abad[i] << "℃," << spine_data->tau_hip[i] << "℃,"
                << spine_data->tau_knee[i] << "℃";
      double warning_level_1 = 102.0;
      double warning_level_2 = 107.0;
      double warning_level_3 = 112.0;
      if (((spine_data->tau_knee[i] > warning_level_1) || (spine_data->tau_hip[i] > warning_level_1) || (spine_data->tau_abad[i] > warning_level_1) ||
           (spine_data->qd_knee[i] > warning_level_1) || (spine_data->qd_hip[i] > warning_level_1) || (spine_data->qd_abad[i] > warning_level_1)) &&
          leds_state != 4 && leds_state != 2 && leds_state != 7) {  // tua is driver;qd is motor;
        // data->abnormal_warning_level_motor = 1;
        // LOG_INFO("[SPI] RED LED ON! MOTOR OR DRIVER WAS OVER %f℃,ABNORMAL WARNING LEVEL WAS EQUAL TO 1 \n", warning_level_1);
        LOG(INFO) << "[SPI] RED LED ON! MOTOR OR DRIVER WAS OVER %f℃,ABNORMAL WARNING LEVEL WAS EQUAL TO 1 \n" << warning_level_1;
      }
      if (((spine_data->tau_knee[i] > warning_level_2) || (spine_data->tau_hip[i] > warning_level_2) || (spine_data->tau_abad[i] > warning_level_2) ||
           (spine_data->qd_knee[i] > warning_level_2) || (spine_data->qd_hip[i] > warning_level_2) || (spine_data->qd_abad[i] > warning_level_2))) {
        // if (leds_state != 4 && leds_state != 2) leds_state = 5;
        // data->abnormal_warning_level_motor = 2;
        // LOG_INFO("[SPI] RED LED ON! MOTOR OR DRIVER WAS OVER %f℃,ABNORMAL WARNING LEVEL WAS EQUAL TO 2 \n", warning_level_2);
        LOG(INFO) << "[SPI] RED LED ON! MOTOR OR DRIVER WAS OVER %f℃,ABNORMAL WARNING LEVEL WAS EQUAL TO 2 \n" << warning_level_2;
      }
      if ((spine_data->tau_knee[i] > warning_level_3 || spine_data->tau_hip[i] > warning_level_3 || spine_data->tau_abad[i] > warning_level_3 ||
           spine_data->qd_knee[i] > warning_level_3 || spine_data->qd_hip[i] > warning_level_3 || spine_data->qd_abad[i] > warning_level_3)) {
        // if (leds_state != 4 && leds_state != 2) leds_state = 6;
        // data->abnormal_warning_level_motor = 3;
        // LOG_INFO("[SPI] RED LED ON! MOTOR OR DRIVER WAS OVER %f℃,ABNORMAL WARNING LEVEL WAS EQUAL TO 3 \n", warning_level_3);
        LOG(INFO) << "[SPI] RED LED ON! MOTOR OR DRIVER WAS OVER %f℃,ABNORMAL WARNING LEVEL WAS EQUAL TO 3 \n" << warning_level_3;
      }
      if ((spine_data->tau_knee[i] > 114 || spine_data->tau_hip[i] > 114 || spine_data->tau_abad[i] > 114 || spine_data->qd_knee[i] > 114 ||
           spine_data->qd_hip[i] > 114 || spine_data->qd_abad[i] > 114) &&
          leds_state != 4 && leds_state != 2) {
        leds_state = 7;
        data->flags[i + stm32_num] = 0xC001;
        LOG_INFO("[SPI]data->flags=0xC001,RED LED ON! TEMPERATURE OVER 114° LEADS TO JOINT PROTECTION\n");
      }
      // warning_level_max = fmax(warning_level_max, data->abnormal_warning_level_motor);
      // data->abnormal_warning_level_motor = warning_level_max;
      if ((uint16_t)screen_info.soc < 5) {
        data->abnormal_warning_level_soc = 3;
      } else if ((uint16_t)screen_info.soc < 10) {
        data->abnormal_warning_level_soc = 2;
      } else if ((uint16_t)screen_info.soc < 15) {
        data->abnormal_warning_level_soc = 1;
      }
      double temperature_group[24];                                          // i[0,1],stm32_num[0,2]
      temperature_group[6 * (i + stm32_num)] = spine_data->qd_abad[i];       // a关节温度
      temperature_group[6 * (i + stm32_num) + 1] = spine_data->qd_hip[i];    // h关节温度
      temperature_group[6 * (i + stm32_num) + 2] = spine_data->qd_knee[i];   // k关节温度
      temperature_group[6 * (i + stm32_num) + 3] = spine_data->tau_abad[i];  // a关节驱动器温度
      temperature_group[6 * (i + stm32_num) + 4] = spine_data->tau_hip[i];   // h关节驱动器温度
      temperature_group[6 * (i + stm32_num) + 5] = spine_data->tau_knee[i];  // k关节驱动器温度
      temperature_group_max = *std::max_element(temperature_group, temperature_group + 24);
      // std::cout << "abnormal_warning_level_motor:" << data->abnormal_warning_level_motor << endl;
      if (temperature_group_max > warning_level_3 && data->abnormal_warning_level_motor == 2) {
        data->abnormal_warning_level_motor = 3;
        if (leds_state != 4 && leds_state != 2) leds_state = 6;
      } else if (temperature_group_max > warning_level_2 && data->abnormal_warning_level_motor == 1) {
        data->abnormal_warning_level_motor = 2;
        if (leds_state != 4 && leds_state != 2 && leds_state != 7) leds_state = 5;
      } else if (temperature_group_max > warning_level_1 && data->abnormal_warning_level_motor == 0) {
        data->abnormal_warning_level_motor = 1;
      } else {
        double temp_offset = 2.5;
        if (temperature_group_max <= warning_level_1 - temp_offset && data->abnormal_warning_level_motor == 1) {
          data->abnormal_warning_level_motor = 0;
        } else if (temperature_group_max <= warning_level_2 - temp_offset && data->abnormal_warning_level_motor == 2) {
          data->abnormal_warning_level_motor = 1;
          if (leds_state == 5) leds_state = 0;
        } else if (temperature_group_max <= warning_level_3 - temp_offset && data->abnormal_warning_level_motor == 3) {
          data->abnormal_warning_level_motor = 2;
          if (leds_state != 4 && leds_state != 2 && leds_state == 6) leds_state = 5;
        }
      }

#ifdef DEBUG_SPI
      motor_temperature[3 * (i + stm32_num)] = spine_data->qd_abad[i];
      motor_temperature[3 * (i + stm32_num) + 1] = spine_data->qd_hip[i];
      motor_temperature[3 * (i + stm32_num) + 2] = spine_data->qd_knee[i];
      driver_temperature[3 * (i + stm32_num)] = spine_data->tau_abad[i];
      driver_temperature[3 * (i + stm32_num) + 1] = spine_data->tau_hip[i];
      driver_temperature[3 * (i + stm32_num) + 2] = spine_data->tau_knee[i];
      // bms_msg
      BMS_msg[0] = 1.0 * screen_info.cur / 100;
      BMS_msg[1] = 1.0 * screen_info.vol / 1000;
      BMS_msg[2] = (1.0 * screen_info.cur / 100) * (1.0 * screen_info.vol / 1000);
      BMS_msg[3] = (uint16_t)screen_info.soc;
      BMS_msg[4] = (int16_t)screen_info.temp;
      BMS_msg[5] = screen_info.state;
      motor_warning_level = data->abnormal_warning_level_motor;
      if (i + stm32_num == 3) {
        rospub.clearData();
        // 0-11 motor temperature
        for (int i = 0; i < 12; i++) {
          rospub.appendData(motor_temperature[i]);
        }
        // 12-23 drive temperature
        for (int i = 0; i < 12; i++) {
          rospub.appendData(driver_temperature[i]);
        }
        // 24-29 BMS msg
        for (int j = 0; j < 6; j++) {
          rospub.appendData(BMS_msg[j]);
        }
        // 30-31 abnormal_warning_level
        rospub.appendData(motor_warning_level);
        rospub.appendData(soc_warning_level);
        rospub.publish();
      }
#endif

    } else {
      data->q_abad[i + stm32_num] = spine_data->q_abad[i] * abad_side_sign[i + stm32_num] + abad_offset[i + stm32_num];
      data->q_hip[i + stm32_num] = spine_data->q_hip[i] * hip_side_sign[i + stm32_num] + hip_offset[i + stm32_num];
      data->q_knee[i + stm32_num] = spine_data->q_knee[i] * knee_side_sign[i + stm32_num] + knee_offset[i + stm32_num];

      data->qd_abad[i + stm32_num] = spine_data->qd_abad[i] * abad_side_sign[i + stm32_num];
      data->qd_hip[i + stm32_num] = spine_data->qd_hip[i] * hip_side_sign[i + stm32_num];
      data->qd_knee[i + stm32_num] = spine_data->qd_knee[i] * knee_side_sign[i + stm32_num];

      data->tau_abad[i + stm32_num] = spine_data->tau_abad[i] * abad_side_sign[i + stm32_num];
      data->tau_hip[i + stm32_num] = spine_data->tau_hip[i] * hip_side_sign[i + stm32_num];
      data->tau_knee[i + stm32_num] = spine_data->tau_knee[i] * knee_side_sign[i + stm32_num];

      // LOG_INFO("data->leg[%d], q_knee=[%f], qd_knee=[%f], tau_knee=[%f]\n", i + stm32_num, data->q_knee[i + stm32_num], data->qd_knee[i +
      // stm32_num],
      //          data->tau_knee[i + stm32_num]);

      //记录电机故障置位；记录读取到的故障码
      if (data->flags[i + stm32_num]) {
        //用于标识是否已经记录故障及故障码，否则会重复记录导致日志变大，初始化为4，取值范围0~3；
        static int olderrorleg[11][4] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                                         4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
        if (data->flags[i + stm32_num] < 0x10) {
          errcode |= 0x001;  //置位显示系统故障
          if (leds_state != 2) {
            leds_state = 2;
            LOG_INFO("[SPI] LED_YELLOW   ON! SYS ERROR\n");
          }

        } else if (data->flags[i + stm32_num] < 0x100000) {
          errcode |= 0x100;  //置位显示电机故障
          if (leds_state != 4) {
            leds_state = 4;
            LOG_INFO("[SPI] LED_RED  ON! MOTOR ERROR \n");
          }
        }
        if (errcode && errcode != errcode_old) {
          LOG(ERROR) << "errcode = " << errcode << "; data->flags[" << i + stm32_num << "]=" << (0xfff & data->flags[i + stm32_num])
                     << " ; I(A) = " << 1.0f * screen_info.cur / 100;
          errcode_old = errcode;
        }
        switch (data->flags[i + stm32_num]) {
          //***系统故障****
          case 0x01:
            if (olderrorleg[6][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "Joint A miss" << i + stm32_num;
              olderrorleg[6][i + stm32_num] = i + stm32_num;
            }
            break;
          case 0x02:
            if (olderrorleg[7][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "Joint H miss" << i + stm32_num;
              olderrorleg[7][i + stm32_num] = i + stm32_num;
            }
            break;
          case 0x04:
            if (olderrorleg[8][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "Joint k miss" << i + stm32_num;
              olderrorleg[8][i + stm32_num] = i + stm32_num;
            }
            break;
          case 0x08:
            if (olderrorleg[9][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "busoff fault" << i + stm32_num;
              olderrorleg[9][i + stm32_num] = i + stm32_num;
            }
            break;
          case 0x0A:
            if (olderrorleg[10][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "spi crc fault" << i + stm32_num;
              olderrorleg[10][i + stm32_num] = i + stm32_num;
            }
            break;

            //*********1.收到故障信号********
          case 0x10:
            if (olderrorleg[0][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "Joint A fault" << i + stm32_num;
              olderrorleg[0][i + stm32_num] = i + stm32_num;
            }
            break;
          case 0x20:
            if (olderrorleg[1][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "Joint H fault" << i + stm32_num;
              olderrorleg[1][i + stm32_num] = i + stm32_num;
            }
            break;
          case 0x40:
            if (olderrorleg[2][i + stm32_num] != i + stm32_num) {
              LOG(ERROR) << "Joint K fault" << i + stm32_num;
              olderrorleg[2][i + stm32_num] = i + stm32_num;
            }
            break;
            //********2.已经读取到故障码********
          case 0x0110:
            if (olderrorleg[3][i + stm32_num] != i + stm32_num) {
              std::bitset<sizeof(float) * 8> bitsA(*reinterpret_cast<unsigned long *>(&data->q_abad[i + stm32_num]));
              LOG(ERROR) << "M_A_ERRCODE_" << i + stm32_num << "=0x" << std::hex << bitsA.to_ullong();
              olderrorleg[3][i + stm32_num] = i + stm32_num;
              // TODO STOP48v
            }
            break;
          case 0x0220:
            if (olderrorleg[4][i + stm32_num] != i + stm32_num) {
              std::bitset<sizeof(float) * 8> bitsH(*reinterpret_cast<unsigned long *>(&data->q_hip[i + stm32_num]));
              LOG(ERROR) << "M_H_ERRCODE_" << i + stm32_num << "=0x" << std::hex << bitsH.to_ullong();
              olderrorleg[4][i + stm32_num] = i + stm32_num;
              // TODO STOP48v
            }
            break;
          case 0x0440:
            if (olderrorleg[5][i + stm32_num] != i + stm32_num) {
              std::bitset<sizeof(float) * 8> bitsK(*reinterpret_cast<unsigned long *>(&data->q_knee[i + stm32_num]));
              LOG(ERROR) << "M_K_ERRCODE_" << i + stm32_num << "=0x" << std::hex << bitsK.to_ullong();
              olderrorleg[5][i + stm32_num] = i + stm32_num;
              // TODO STOP48v
            }
            break;
          default:
            break;
        }
      }
    }
  }
#if 1
#ifdef DEBUG_SPI
  static int num_error_qA[4] = {0, 0, 0, 0};
  static int num_error_qH[4] = {0, 0, 0, 0};
  static int num_error_qK[4] = {0, 0, 0, 0};
  static int num_error_qdA[4] = {0, 0, 0, 0};
  static int num_error_qdH[4] = {0, 0, 0, 0};
  static int num_error_qdK[4] = {0, 0, 0, 0};
  static int num_error_qddA[4] = {0, 0, 0, 0};
  static int num_error_qddH[4] = {0, 0, 0, 0};
  static int num_error_qddK[4] = {0, 0, 0, 0};
  static int num_error_tauA[4] = {0, 0, 0, 0};
  static int num_error_tauH[4] = {0, 0, 0, 0};
  static int num_error_tauK[4] = {0, 0, 0, 0};
  static int num_error_continuous[4] = {0, 0, 0, 0};
  // static int num_error_itr[4] = {0, 0, 0, 0};
  // static int num_error_count[4] = {0, 0, 0, 0};
#endif
  // TODO spinor
  for (int i = 0; i < 2; i++) {
    // #ifdef DEBUG_SPI
    //     // num_error_itr[stm32_num + i]++;
    //     // if (num_error_itr[stm32_num + i] >= 10000) {
    //     //   LOG_INFO("\n**** SPI ERROR RATE PER 10000 COUNT: LEG[%d] num_error=[%d]\n ", stm32_num + i,
    //     num_error_count[stm32_num + i]);
    //     //   num_error_itr[stm32_num + i] = 0;
    //     //   num_error_count[stm32_num + i] = 0;
    //     // }
    // #endif
    // clang-format off
    if (!((data->q_abad[stm32_num+i] < -1.2f) //0.78539815
        ||(data->q_abad[stm32_num+i] > 1.2f) 
        ||(data->q_hip[stm32_num+i] < -3.0f)  //2.618
        ||(data->q_hip[stm32_num+i] > 2.100001f) //0.100001f
        ||(data->q_knee[ stm32_num+i] < 0.0f) //0.5446
        ||(data->q_knee[ stm32_num+i] > 3.2f) //2.5516
        || (data->qd_abad[stm32_num+i] < -60.0f) 
        || (data->qd_abad[stm32_num+i] > 60.0f) 
        || (data->qd_hip[stm32_num+i] < -60.0f) 
        || (data->qd_hip[stm32_num+i] > 60.0f) 
        || (data->qd_knee[stm32_num+i] < -60.0f) 
        || (data->qd_knee[stm32_num+i] > 60.0f)
        // || (abs(data->qd_abad[stm32_num+i] - data_old.qd_abad[stm32_num+i]) > 10) //5000*0.002
        // || (abs(data->qd_hip[stm32_num+i] - data_old.qd_hip[stm32_num+i]) > 10)
        // || (abs(data->qd_knee[stm32_num+i] - data_old.qd_knee[stm32_num+i]) > 10)
        // || (abs(data->tau_abad[stm32_num+i] - data_old.tau_abad[stm32_num+i]) > 40) //5000*0.002
        // || (abs(data->tau_hip[stm32_num+i] - data_old.tau_hip[stm32_num+i]) > 40)
        // || (abs(data->tau_knee[stm32_num+i] - data_old.tau_knee[stm32_num+i]) > 40)
        )) {
      // clang-format on
      data_old.q_abad[stm32_num + i] = data->q_abad[stm32_num + i];
      data_old.q_hip[stm32_num + i] = data->q_hip[stm32_num + i];
      data_old.q_knee[stm32_num + i] = data->q_knee[stm32_num + i];
      data_old.qd_abad[stm32_num + i] = data->qd_abad[stm32_num + i];
      data_old.qd_hip[stm32_num + i] = data->qd_hip[stm32_num + i];
      data_old.qd_knee[stm32_num + i] = data->qd_knee[stm32_num + i];
      data_old.tau_abad[stm32_num + i] = data->tau_abad[stm32_num + i];
      data_old.tau_hip[stm32_num + i] = data->tau_hip[stm32_num + i];
      data_old.tau_knee[stm32_num + i] = data->tau_knee[stm32_num + i];

// test code
#ifdef DEBUG_SPI
      num_error_qA[stm32_num + i] = 0;
      num_error_qH[stm32_num + i] = 0;
      num_error_qK[stm32_num + i] = 0;
      num_error_qdA[stm32_num + i] = 0;
      num_error_qdH[stm32_num + i] = 0;
      num_error_qdK[stm32_num + i] = 0;
      num_error_qddA[stm32_num + i] = 0;
      num_error_qddH[stm32_num + i] = 0;
      num_error_qddK[stm32_num + i] = 0;
      // num_error_tauA[stm32_num + i] = 0;
      // num_error_tauH[stm32_num + i] = 0;
      // num_error_tauK[stm32_num + i] = 0;
      num_error_continuous[stm32_num + i] = 0;
#endif
      // data_old.tau_knee[stm32_num] = data->tau_knee[stm32_num];
    } else {
// check position error
#ifdef DEBUG_SPI
      /*check joint position error*/
      if ((data->q_abad[stm32_num + i] < -1.2f) || (data->q_abad[stm32_num + i] > 1.2f)) {
        num_error_qA[stm32_num + i] = num_error_qA[stm32_num + i] + 1;
        if (num_error_qA[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI q_abad abs ERROR [%d] num_error=[%d] q_abad=[%f]\n ", stm32_num + i, num_error_qA[stm32_num + i],
                   data->q_abad[stm32_num + i]);
        }
      } else {
        num_error_qA[stm32_num + i] = 0;
      }
      if ((data->q_hip[stm32_num + i] < -3.0f) || (data->q_hip[stm32_num + i] > 2.0f)) {  // 0.200001f
        num_error_qH[stm32_num + i] = num_error_qH[stm32_num + i] + 1;
        if (num_error_qH[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI q_hip abs ERROR [%d] num_error=[%d] q_hip=[%f]\n ", stm32_num + i, num_error_qH[stm32_num + i],
                   data->q_hip[stm32_num + i]);
        }
      } else {
        num_error_qH[stm32_num + i] = 0;
      }
      if ((data->q_knee[stm32_num + i] < 0.0f) || (data->q_knee[stm32_num + i] > 3.2f)) {
        num_error_qK[stm32_num + i] = num_error_qK[stm32_num + i] + 1;
        if (num_error_qK[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI q_knee abs ERROR [%d] num_error=[%d] q_knee=[%f]\n ", stm32_num + i, num_error_qK[stm32_num + i],
                   data->q_knee[stm32_num + i]);
        }
      } else {
        num_error_qK[stm32_num + i] = 0;
      }

      /*check joint velocity error*/
      if ((data->qd_abad[stm32_num + i] < -20.0f) || (data->qd_abad[stm32_num + i] > 20.0f)) {
        num_error_qdA[stm32_num + i] = num_error_qdA[stm32_num + i] + 1;
        if (num_error_qdA[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI qd_abad abs ERROR [%d] num_error=[%d] qd_abad=[%f]\n ", stm32_num + i, num_error_qdA[stm32_num + i],
                   data->qd_abad[stm32_num + i]);
        }
      } else {
        num_error_qdA[stm32_num + i] = 0;
      }
      if ((data->qd_hip[stm32_num + i] < -20.0f) || (data->qd_hip[stm32_num + i] > 20.0f)) {
        num_error_qdH[stm32_num + i] = num_error_qdH[stm32_num + i] + 1;
        if (num_error_qdH[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI qd_hip abs ERROR [%d] num_error=[%d] qd_hip=[%f]\n ", stm32_num + i, num_error_qdH[stm32_num + i],
                   data->qd_hip[stm32_num + i]);
        }
      } else {
        num_error_qdH[stm32_num + i] = 0;
      }
      if ((data->qd_knee[stm32_num + i] < -20.0f) || (data->qd_knee[stm32_num + i] > 20.0f)) {
        num_error_qdK[stm32_num + i] = num_error_qdK[stm32_num + i] + 1;
        if (num_error_qdK[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI qd_knee abs ERROR [%d] num_error=[%d] qd_knee=[%f]\n ", stm32_num + i, num_error_qdK[stm32_num + i],
                   data->qd_knee[stm32_num + i]);
        }
      } else {
        num_error_qdK[stm32_num + i] = 0;
      }
      /*check joint acceleration error*/
      if (abs(data->qd_abad[stm32_num + i] - data_old.qd_abad[stm32_num + i]) > 10) {  // 8000.0f * 0.002
        num_error_qddA[stm32_num + i] = num_error_qddA[stm32_num + i] + 1;
        if (num_error_qddA[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI qdd_A abs ERROR [%d] num_error=[%d] qd_abad=[%f] qd_abad_old=[%f]\n ", stm32_num + i, num_error_qddA[stm32_num + i],
                   data->qd_abad[stm32_num + i], data_old.qd_abad[stm32_num + i]);
        }
      } else {
        num_error_qddA[stm32_num + i] = 0;
      }
      // delta velocity H
      if (abs(data->qd_hip[stm32_num + i] - data_old.qd_hip[stm32_num + i]) > 10) {
        num_error_qddH[stm32_num + i] = num_error_qddH[stm32_num + i] + 1;
        if (num_error_qddH[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI qdd_H abs ERROR [%d] num_error=[%d] qd_hip=[%f] qd_hip_old=[%f]\n ", stm32_num + i, num_error_qddH[stm32_num + i],
                   data->qd_hip[stm32_num + i], data_old.qd_hip[stm32_num + i]);
        }
      } else {
        num_error_qddH[stm32_num + i] = 0;
      }
      // delta velocity K
      if (abs(data->qd_knee[stm32_num + i] - data_old.qd_knee[stm32_num + i]) > 10) {
        num_error_qddK[stm32_num + i] = num_error_qddK[stm32_num + i] + 1;
        if (num_error_qddK[stm32_num + i] > 10) {
          LOG_INFO("\n**** SPI qdd_K abs ERROR [%d] num_error=[%d] qd_knee=[%f] qd_knee_old=[%f]\n ", stm32_num + i, num_error_qddK[stm32_num + i],
                   data->qd_knee[stm32_num + i], data_old.qd_knee[stm32_num + i]);
        }
      } else {
        num_error_qddK[stm32_num + i] = 0;
      }
      // /*check errors of joint torques*/
      // if (abs(data->tau_abad[stm32_num + i] - data_old.tau_abad[stm32_num + i]) > 40) {
      //   num_error_tauA[stm32_num + i] += 1;
      //   if (num_error_tauA[stm32_num + i] > 10) {
      //     LOG_INFO("\n**** SPI tau_A delta ERROR [%d] num_error=[%d] tau_A=[%f] tau_A_old=[%f]\n ", stm32_num +
      //     i, num_error_tauA[stm32_num + i],
      //            data->tau_abad[stm32_num + i], data_old.tau_abad[stm32_num + i]);
      //   }
      // } else {
      //   num_error_tauA[stm32_num + i] = 0;
      // }
      // if (abs(data->tau_hip[stm32_num + i] - data_old.tau_hip[stm32_num + i]) > 40) {
      //   num_error_tauH[stm32_num + i] += 1;
      //   if (num_error_tauH[stm32_num + i] > 10) {
      //     LOG_INFO("\n**** SPI tau_H delta ERROR [%d] num_error=[%d] tau_hip=[%f] tau_hip_old=[%f]\n ", stm32_num
      //     + i, num_error_tauH[stm32_num + i],
      //            data->tau_hip[stm32_num + i], data_old.tau_hip[stm32_num + i]);
      //   }
      // } else {
      //   num_error_tauH[stm32_num + i] = 0;
      // }
      // if (abs(data->tau_knee[stm32_num + i] - data_old.tau_knee[stm32_num + i]) > 40) {
      //   num_error_tauK[stm32_num + i] += 1;
      //   if (num_error_tauK[stm32_num + i] > 10) {
      //     LOG_INFO("\n**** SPI tau_H delta ERROR [%d] num_error=[%d] tau_knee=[%f] tau_knee_old=[%f]\n ",
      //     stm32_num + i, num_error_tauK[stm32_num + i],
      //            data->tau_knee[stm32_num + i], data_old.tau_knee[stm32_num + i]);
      //   }
      // } else {
      //   num_error_tauK[stm32_num + i] = 0;
      // }

      // num_error_continuous[stm32_num + i]++;
      // if (num_error_continuous[stm32_num + i] > 200) {
      //   LOG_INFO("\n**** SPI num_error_continuous ERROR [%d] num_error=[%d] \n ", stm32_num + i, num_error_continuous[stm32_num + i]);
      // }
      // num_error_count[stm32_num + i]++;
#endif

      data->q_abad[stm32_num + i] = data_old.q_abad[stm32_num + i];
      data->q_hip[stm32_num + i] = data_old.q_hip[stm32_num + i];
      data->q_knee[stm32_num + i] = data_old.q_knee[stm32_num + i];

      data->qd_abad[stm32_num + i] = data_old.qd_abad[stm32_num + i];
      data->qd_hip[stm32_num + i] = data_old.qd_hip[stm32_num + i];
      data->qd_knee[stm32_num + i] = data_old.qd_knee[stm32_num + i];

      data->tau_abad[stm32_num + i] = data_old.tau_abad[stm32_num + i];
      data->tau_hip[stm32_num + i] = data_old.tau_hip[stm32_num + i];
      data->tau_knee[stm32_num + i] = data_old.tau_knee[stm32_num + i];
    }
  }

#endif
}

static void hex_dump(const void *src, size_t length, size_t line_size, char *prefix) {
  int i = 0;
  const unsigned char *address = (const unsigned char *)src;
  const unsigned char *line = address;
  unsigned char c;

  LOG_INFO("%s | ", prefix);
  while (length-- > 0) {
    LOG_INFO("%02X ", *address++);
    if (!(++i % line_size) || (length == 0 && i % line_size)) {
      if (length == 0) {
        while (i++ % line_size) LOG_INFO("__ ");
      }
      LOG_INFO(" |");
      while (line < address) {
        c = *line++;
        LOG_INFO("%c", (c < 32 || c > 126) ? '.' : c);
      }
      LOG_INFO("|\n");
      if (length > 0) LOG_INFO("%s | ", prefix);
    }
  }
}
/*!
 * send receive data and command from spine
 */
void spi_send_receive(spi_command_t *command, spi_data_t *data) {
  uint16_t tx_buf[K_WORDS_PER_MESSAGE];
  uint16_t rx_buf[K_WORDS_PER_MESSAGE];

  spi_driver_iterations++;
  data->spi_driver_status = spi_driver_iterations << 16;

  for (int spi_board = 0; spi_board < 2; spi_board++) {
    spi_to_spine(command, &g_spine_cmd, spi_board * 2);
    // pointers to command/data spine array
    uint16_t *cmd_d = (uint16_t *)&g_spine_cmd;
    uint16_t *data_d = (uint16_t *)&g_spine_data;
    // zero rx buffer
    memset(rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));
    // copy into tx buffer flipping bytes
    for (int i = 0; i < K_WORDS_PER_MESSAGE; i++) tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);
    // each word is two bytes long
    size_t word_len = 2;  // 16 bit word
    // spi message struct
    struct spi_ioc_transfer spi_message;
    // zero message struct.
    memset(&spi_message, 0, sizeof(spi_ioc_transfer));
    // set up message struct
    spi_message.bits_per_word = spi_bits_per_word;
    spi_message.cs_change = 1;
    spi_message.delay_usecs = 0;
    spi_message.len = word_len * K_WORDS_PER_MESSAGE;
    spi_message.rx_buf = (uint64_t)rx_buf;
    spi_message.tx_buf = (uint64_t)tx_buf;
    // hex_dump(tx_buf, 136, 20, "TX");
    // do spi communication
    int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd, SPI_IOC_MESSAGE(1), &spi_message);
    (void)rv;

    // hex_dump(rx_buf, 92, 16, "RX");//92-8=84  /2=42
    // flip bytes the other way
    // 首个U16不稳定，考虑到实际使用的稳定性，再将从机发送数据向后移3个U16。总长为46 U16
    // 即实际前2个U32为填充，真实数据从第3个U32开始，长度42个U16；
    // 同时发送也做首个U32视作pad处理，例如测试填充了0x41414141,即4个字符A，
    // 实际从机接受到的数据从第2个U32开始。

    for (int i = 0; i < 42 + 2 + 2; i++)  // 30
      data_d[i] = rx_buf[i + 2 + 2];
    // hex_dump(data_d, 84, 20, dev);//84/2 = 42
    spine_to_spi(data, &g_spine_data, spi_board * 2);
  }
}

/*!
 * Run SPI
 */
void spi_driver_run() {
  pthread_mutex_lock(&spi_mutex);
  static int cnt1s = 1;  // 1
  if (cnt1s++ % 500 == 0) {
    temperatureget = 1;
    cnt1s = 1;
  }
  spi_send_receive(&spi_command_drv, &spi_data_drv);
  if (temperatureget) {
    temperatureget = 0;
  }
  pthread_mutex_unlock(&spi_mutex);
}
// /*!
//  * send receive data t and command from spine
//  */
// void spi_send_receive_t(spi_command_t *command, spi_data_t *data) {
//   char *timeline;
//   uint16_t tx_buf[K_WORDS_PER_MESSAGE];
//   uint16_t rx_buf[K_WORDS_PER_MESSAGE];

//   spi_driver_iterations++;
//   data->spi_driver_status = spi_driver_iterations << 16;

//   // transmit and receive buffers

//   for (int spi_board = 0; spi_board < 2; spi_board++) {
//     spi_to_spine(command, &g_spine_cmd, spi_board * 2);
//     uint16_t *cmd_d = (uint16_t *)&g_spine_cmd;
//     uint16_t *data_d = (uint16_t *)&g_spine_data;
//     // zero rx buffer
//     memset(rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));
//     // copy into tx buffer flipping bytes
//     for (int i = 0; i < K_WORDS_PER_MESSAGE; i++)
//       // tx_buf[i] = cmd_d[i];
//       tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);
//     // each word is two bytes long
//     size_t word_len = 2;  // 16 bit word
//     // spi message struct
//     struct spi_ioc_transfer spi_message;
//     // zero message struct.
//     memset(&spi_message, 0, sizeof(spi_ioc_transfer));
//     // set up message struct
//     spi_message.bits_per_word = spi_bits_per_word;
//     spi_message.cs_change = 1;
//     spi_message.delay_usecs = 0;
//     spi_message.len = word_len * K_WORDS_PER_MESSAGE;
//     spi_message.rx_buf = (uint64_t)rx_buf;
//     spi_message.tx_buf = (uint64_t)tx_buf;
//     int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd, SPI_IOC_MESSAGE(1), &spi_message);
//     (void)rv;

//     // hex_dump(rx_buf, 92, 16, "RX");//92-8=84  /2=42
//     // flip bytes the other way
//     // 首个U16不稳定，考虑到实际使用的稳定性，再将从机发送数据向后移3个U16。总长为46 U16
//     // 即实际前2个U32为填充，真实数据从第3个U32开始，长度42个U16；
//     // 同时发送也做首个U32视作pad处理，例如测试填充了0x41414141,即4个字符A，
//     // 实际从机接受到的数据从第2个U32开始。

//     for (int i = 0; i < 42 + 2 + 2; i++)  // 30
//       data_d[i] = rx_buf[i + 2 + 2];
//     spine_to_spi(data, &g_spine_data, spi_board * 2);
//   }
// }

/*!
 * Run temperature
 */
// void temperature(spi_command_t *tcommand, spi_data_t *tdata) {
//   pthread_mutex_lock(&spi_mutex);
//   spi_send_receive_t(tcommand, tdata);
//   pthread_mutex_unlock(&spi_mutex);
// }

/*!
 * Get the spi command
 */
spi_command_t *get_spi_command() { return &spi_command_drv; }

/*!
 * Get the spi data
 */
spi_data_t *get_spi_data() { return &spi_data_drv; }
