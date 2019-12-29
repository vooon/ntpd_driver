/** ROS 2 Composable node for ShmDriver
 *
 * This node will subscribe to any TimeReference message
 * and sends it's data to ntpd.
 *
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @license BSD
 *
 * Copyright 2019 Vladimir Ermakov.
 * Based on ntpshm.c from gpsd.
 *
 * @file
 */

#pragma once

#include <chrono>
#include <memory>
#include <functional>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>


/** the definition of shmTime is from ntpd source ntpd/refclock_shm.c */
struct shmTime
{
  int mode; /* 0 - if valid set
             *       use values,
             *       clear valid
             * 1 - if valid set
             *       if count before and after read of values is equal,
             *         use values
             *       clear valid
             */
  volatile int count;
  time_t clockTimeStampSec;
  int clockTimeStampUSec;
  time_t receiveTimeStampSec;
  int receiveTimeStampUSec;
  int leap;
  int precision;
  int nsamples;
  volatile int valid;
  unsigned        clockTimeStampNSec;     /* Unsigned ns timestamps */
  unsigned        receiveTimeStampNSec;   /* Unsigned ns timestamps */
  int             dummy[8];
};

const long int NTPD_SHM_BASE = 0x4e545030;

using ShmTimeT = volatile struct shmTime;


class ShmDriver : public rclcpp::Node
{
  public:
    ShmDriver();

  private:
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_sub_;

    int shm_unit_;
    bool fixup_date_;
    std::string time_ref_topic;

    std::unique_ptr<ShmTimeT, std::function<void(ShmTimeT*)>> shm_;

    void time_ref_cb(const sensor_msgs::msg::TimeReference::SharedPtr msg);
    void set_system_time(const double seconds);

    ShmTimeT* attach_shmTime(int unit);
    void detach_shmTime(ShmTimeT* shm);
};
