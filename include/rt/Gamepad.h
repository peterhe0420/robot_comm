#ifndef GAMEPAD_H
#define GAMEPAD_H
// clang-format off
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include "lcm-types/cpp/gamepad_lcmt.hpp"
// clang-format on

class Gamepad {
 private:
  std::shared_ptr<gamepad_lcmt> gamepad_lcmt_msgptr_;
  std::thread ros_thread_;
  // ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

 public:
  Gamepad();
  void init();
  std::shared_ptr<gamepad_lcmt> getGamepadPtr() { return gamepad_lcmt_msgptr_; };

  // for ros joy
  void JoySignalCallback(const sensor_msgs::JoyConstPtr msg);
};

#endif