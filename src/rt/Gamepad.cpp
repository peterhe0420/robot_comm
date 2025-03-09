#include "rt/Gamepad.h"

Gamepad::Gamepad() { gamepad_lcmt_msgptr_ = std::make_shared<gamepad_lcmt>(); };

void Gamepad::init() {
  // std::cout << "run Gamepad" << std::endl;
  // std::cout << "run Gamepad" << std::endl;
  int a{0};
  ros::init(a, NULL, "nh_gamepad");
  ros::NodeHandle nh_;
  joy_sub_ = nh_.subscribe("/joy", 1, &Gamepad::JoySignalCallback, this);
  std::function<void()> f_ros_spin = [this]() { ros::spin(); };
  std::thread ros_spin_thread_(f_ros_spin);
  ros_spin_thread_.detach();
}

void Gamepad::JoySignalCallback(const sensor_msgs::JoyConstPtr msg) {
  gamepad_lcmt_msgptr_->a = msg->buttons[0];
  gamepad_lcmt_msgptr_->b = msg->buttons[1];
  gamepad_lcmt_msgptr_->x = msg->buttons[2];
  gamepad_lcmt_msgptr_->y = msg->buttons[3];
  gamepad_lcmt_msgptr_->leftTriggerButton = msg->buttons[4];
  gamepad_lcmt_msgptr_->rightTriggerButton = msg->buttons[5];
  gamepad_lcmt_msgptr_->back = msg->buttons[6];
  gamepad_lcmt_msgptr_->start = msg->buttons[7];
  // bei-tong Gamepad not define
  // gamepad_lcmt_msgptr_->leftBumper = 0;
  // gamepad_lcmt_msgptr_->rightBumper = 0;
  gamepad_lcmt_msgptr_->leftStickButton = msg->buttons[9];
  gamepad_lcmt_msgptr_->rightStickButton = msg->buttons[10];

  gamepad_lcmt_msgptr_->leftStickAnalog[0] = -msg->axes[0];   // x
  gamepad_lcmt_msgptr_->leftStickAnalog[1] = msg->axes[1];    // y
  gamepad_lcmt_msgptr_->leftTriggerAnalog = msg->axes[2];     //
  gamepad_lcmt_msgptr_->rightStickAnalog[0] = -msg->axes[3];  // x
  gamepad_lcmt_msgptr_->rightStickAnalog[1] = msg->axes[4];   // y
  gamepad_lcmt_msgptr_->rightTriggerAnalog = msg->axes[5];    //
  // std::cout << "[Gamepad]gamepad_lcmt_msgptr_->a:" << gamepad_lcmt_msgptr_->a << std::endl;
};
