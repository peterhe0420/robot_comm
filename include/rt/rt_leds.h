/*!
 * @file _rt_leds.h
 * @brief  leds state
 */
// clang-format off
#ifndef _rt_leds
#define _rt_leds


#include <iostream>
#include <JetsonGPIO.h>

#include <fcntl.h>      //Needed for SPI port
#include <sys/ioctl.h>  //Needed for SPI port

// incredibly obscure bug in SPI_IOC_MESSAGE macro is fixed by this
#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

// #include <linux/spi/spidev.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  //Needed for SPI port


#define  LED_YELLOW 31  // 31-11-yellow
#define  LED_GREEN  29  // 29-01-green
#define  LED_RED    15  // 15-12-red


void rt_leds_init();
void rt_leds_state();

#endif
