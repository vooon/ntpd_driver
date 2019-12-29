/** ROS ntpd driver (via shm)
 *
 * This node will subscribe to any TimeReference message
 * and sends it's data to ntpd.
 *
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @license BSD
 *
 * Copyright 2014 Vladimir Ermakov.
 * Based on ntpshm.c from gpsd.
 *
 * @file
 */

#include <ntpd_driver/ShmDriver.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShmDriver>());
  rclcpp::shutdown();
  return 0;
}
