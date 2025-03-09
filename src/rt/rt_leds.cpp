/*LEDS*/
// #include <iostream>
// for delay function.
// #include <chrono>
// #include <thread>

// for signal handling
// #include <JetsonGPIO.h>
// #include <signal.h>

#include "rt/rt_leds.h"
// using namespace std;

int leds_state = 0;
int oldleds_state = 0;
void rt_leds_init() {
  // Pin Setup.
  GPIO::setmode(GPIO::BOARD);
  GPIO::setwarnings(false);
  // set pin as an output pin with optional initial state of LOW
  GPIO::setup(LED_YELLOW, GPIO::OUT, GPIO::LOW);
  GPIO::setup(LED_RED, GPIO::OUT, GPIO::LOW);
  GPIO::setup(LED_GREEN, GPIO::OUT, GPIO::LOW);
}

void rt_leds_state() {
  if (leds_state != oldleds_state) {
    switch (leds_state) {
      case 0:
        // closed
        GPIO::output(LED_GREEN, GPIO::LOW);
        GPIO::output(LED_YELLOW, GPIO::LOW);
        GPIO::output(LED_RED, GPIO::LOW);
        break;
      case 1:
        // start
        GPIO::output(LED_GREEN, GPIO::HIGH);
        GPIO::output(LED_YELLOW, GPIO::LOW);
        GPIO::output(LED_RED, GPIO::LOW);
        break;
      case 2:
        // warning motor
        GPIO::output(LED_GREEN, GPIO::LOW);
        GPIO::output(LED_YELLOW, GPIO::HIGH);
        GPIO::output(LED_RED, GPIO::LOW);
        break;
      case 3:
        // soc<20%
        GPIO::output(LED_GREEN, GPIO::LOW);
        GPIO::output(LED_YELLOW, GPIO::HIGH);
        GPIO::output(LED_RED, GPIO::LOW);
        break;
      case 4:
        // error 置位显示电机故障
        GPIO::output(LED_GREEN, GPIO::LOW);
        GPIO::output(LED_YELLOW, GPIO::LOW);
        GPIO::output(LED_RED, GPIO::HIGH);
        break;
      case 5:
        // high temperature
        GPIO::output(LED_GREEN, GPIO::LOW);
        GPIO::output(LED_YELLOW, GPIO::HIGH);
        GPIO::output(LED_RED, GPIO::LOW);
        break;
      case 6:
        // soc<5%
        GPIO::output(LED_GREEN, GPIO::LOW);
        GPIO::output(LED_YELLOW, GPIO::LOW);
        GPIO::output(LED_RED, GPIO::HIGH);
        break;
      case 7:
        //
        GPIO::output(LED_GREEN, GPIO::HIGH);
        GPIO::output(LED_YELLOW, GPIO::HIGH);
        GPIO::output(LED_RED, GPIO::HIGH);
        break;
      default:
        break;
    }
    oldleds_state = leds_state;
  }
}
