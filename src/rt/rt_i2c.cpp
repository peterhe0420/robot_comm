// #include <stdio.h>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
#include <linux/i2c-dev.h>
// #include "rt/ssd1306_i2c.h"
#include <string.h>

#include <ctime>

#include "rt/oled_fonts.h"
// #include <stdlib>

#include <byteswap.h>
#include <math.h>
#include <pthread.h>

#include "rt/rt_i2c.h"
// #include <lcm/lcm-cpp.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/include/printLog.h"
#include "rt/batMgmt.h"

extern batParamGet screen_info;
extern uint16_t errcode;

// 字体 16

int i2cfd;
time_t rawtime;
struct tm *info;

pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

void send_command(unsigned char command) {
  unsigned char buf[2];
  buf[0] = 0x00;  // 控制字节，0x00表示接下来的字节是命令
  buf[1] = command;
  write(i2cfd, buf, 2);
}

void send_data_8byte(const unsigned char *data) {
  unsigned char buf[9];
  buf[0] = 0x40;  // 控制字节，0x40表示接下来的字节是数据
  memcpy(&buf[1], data, 8);
  write(i2cfd, buf, 9);
}

void oled_set_position(unsigned char x, unsigned char y) {
  //以下3个寄存器只在页寻址的模式下有效
  send_command(0xb0 + y);                  //页地址设置     0xb0~0xb7
  send_command((x & 0x0f));                //列低位地址设置
  send_command(((x & 0xf0) >> 4) | 0x10);  //列高位地址设置
}

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样
void oled_clear(void) {
  unsigned char x, y;
  unsigned char buf[MAX_COLUMN + 1] = {0x40};

  for (y = 0; y < 8; y++) {
    send_command(0xb0 + y);  //设置页地址（0~7）
    send_command(0x00);      //设置显示位置—列低地址
    send_command(0x10);      //设置显示位置—列高地址
    write(i2cfd, buf, MAX_COLUMN + 1);
  }  //更新显示
}
//在指定位置显示一个字符,包括部分字符
// x:0~127，y:0~7
void oled_show_char(unsigned char x, unsigned char y, unsigned char chr) {
  unsigned char c = 0, i = 0;

  c = chr - ' ';  //得到偏移后的值
  if (x > MAX_COLUMN - 1) {
    x = 0;
    y = y + 2;
  }

  oled_set_position(x, y);
  send_data_8byte(&F8x16[c * 16]);  //先写上半部分
  oled_set_position(x, y + 1);
  send_data_8byte(&F8x16[c * 16 + 8]);  //后写下半部分
}

//显示一个字符串
void oled_show_string(unsigned char x, unsigned char y, char *str) {
  unsigned char j = 0;

  while (str[j] != '\0') {
    oled_show_char(x, y, str[j]);
    // if(Char_Size==12)
    // {
    //   x+=6;
    //   if(x>126)
    //   {
    //     x=0;
    //     y+=2;
    //   }
    //   j++;//移动一次就是一个page，取值0-7
    // }
    // else if(Char_Size==16)
    // {
    x += 8;
    if (x > 120) {
      x = 0;
      y += 2;
    }
    j++;  //移动一次就是一个page，取值0-7

    // }
  }
}

void send_name_logo() {
  unsigned char i = 0, n = 0;

  for (n = 0; n < 2; n++) {
    oled_set_position(32 + 8 + 32 * n, 1);
    send_data_8byte(&NameLogo[n * 32]);
    oled_set_position(32 + 16 + 32 * n, 1);
    send_data_8byte(&NameLogo[n * 32 + 8]);
    oled_set_position(32 + 8 + 32 * n, 2);
    send_data_8byte(&NameLogo[n * 32 + 16]);
    oled_set_position(32 + 16 + 32 * n, 2);
    send_data_8byte(&NameLogo[n * 32 + 24]);
  }
  for (n = 2; n < 4; n++) {
    oled_set_position(-32 + 8 + 32 * n, 5);
    send_data_8byte(&NameLogo[n * 32]);
    oled_set_position(-32 + 16 + 32 * n, 5);
    send_data_8byte(&NameLogo[n * 32 + 8]);
    oled_set_position(-32 + 8 + 32 * n, 6);
    send_data_8byte(&NameLogo[n * 32 + 16]);
    oled_set_position(-32 + 16 + 32 * n, 6);
    send_data_8byte(&NameLogo[n * 32 + 24]);
  }
}

void send_tempterature_logo(unsigned char x, unsigned char y) {
  unsigned char i = 0;
  if (x > MAX_COLUMN - 1) {
    x = 0;
    y = y + 2;
  }
  oled_set_position(x, y);
  send_data_8byte(&TemperatureLogo[0]);
  oled_set_position(x, y + 1);
  send_data_8byte(&TemperatureLogo[8]);
}

int screen_init() {
  // int i2cfd;
  // char buf[10];

  // 打开I2C设备
  i2cfd = open(I2C_DEVICE, O_RDWR);
  if (i2cfd < 0) {
    perror("打开设备失败");
    return 1;
  }
  // LOG(INFO) << ("[RT I2C] open ok\n");

  // 设定I2C从设备的地址
  if (ioctl(i2cfd, I2C_SLAVE, SCREEN_ADDRESS) < 0) {
    perror("设定地址失败");
    return 1;
  }
  // LOG(INFO) << ("[RT I2C] ioctl ok\n");

  // 发送命令
  send_command(0xAF);  // 开启显示
  send_command(0x20);  // 设定内存地址模式
  send_command(0x00);  // 选择水平地址模式

  oled_clear();
  send_name_logo();
  LOG(INFO) << ("[RT I2C] init data ok\n");

  return 0;
}

int screen_show_message() {
  static char clearonce = 0;
  char str_bat[5][8];
  if (clearonce == 0) {
    oled_clear();
    clearonce = 1;

    oled_show_string(1 * 8, 4, "ERR:");
    //电机-电池-系统， 电池故障，置位，详情查log。
    static int olderrcode = 0;
    if (screen_info.state && olderrcode != screen_info.state) {
      errcode |= 0x010;
      LOG(WARNING) << "screen_info.state = " << screen_info.state;
      olderrcode = screen_info.state;
    }
    sprintf(str_bat[4], "%03X", errcode);
    oled_show_string(6 * 8, 4, str_bat[4]);

    oled_show_string(1 * 8, 2, "SOC:      %");
    sprintf(str_bat[2], "%d", screen_info.soc);
    oled_show_string(6 * 8, 2, str_bat[2]);

    // #endif
  } else if (clearonce == 1) {
    oled_clear();
    clearonce = 2;
    oled_show_string(1 * 8, 2, "Vol:        V");
    sprintf(str_bat[0], "%.03f", 1.0f * screen_info.vol / 1000);
    oled_show_string(6 * 8, 2, str_bat[0]);
    //电流传送单位为10mA
    oled_show_string(1 * 8, 4, "Cur:        A");
    float current_A = 1.0f * screen_info.cur / 100;
    sprintf(str_bat[1], "%.03f", current_A);
    oled_show_string(6 * 8, 4, str_bat[1]);

  } else if (clearonce == 2) {
    oled_clear();
    clearonce = 3;
    char strcur[2][6];
    oled_show_string(1 * 8, 2, "TMP1:     ");
    sprintf(strcur[0], "%.1f", screen_info.ktemp[0]);
    oled_show_string(6 * 8, 2, strcur[0]);
    send_tempterature_logo(8 * 12, 2);

    oled_show_string(1 * 8, 4, "TMP2:     ");
    sprintf(strcur[1], "%.1f", screen_info.ktemp[1]);
    oled_show_string(6 * 8, 4, strcur[1]);
    send_tempterature_logo(8 * 12, 4);
  } else {
    oled_clear();
    clearonce = 0;
    char strcur[2][6];
    oled_show_string(1 * 8, 2, "TMP3:     ");
    sprintf(strcur[0], "%.1f", screen_info.ktemp[2]);
    oled_show_string(6 * 8, 2, strcur[0]);
    send_tempterature_logo(8 * 12, 2);

    oled_show_string(1 * 8, 4, "TMP4:     ");
    sprintf(strcur[1], "%.1f", screen_info.ktemp[3]);
    oled_show_string(6 * 8, 4, strcur[1]);
    send_tempterature_logo(8 * 12, 4);
  }

  // LOG(INFO) << ("[RT I2C] show message data done\n");
  return 0;
}

void i2c_driver_run() {
  pthread_mutex_lock(&i2c_mutex);
  screen_show_message();
  pthread_mutex_unlock(&i2c_mutex);
}
