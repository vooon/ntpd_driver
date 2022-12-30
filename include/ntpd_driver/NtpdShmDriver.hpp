//
// Copyright (c) 2014-2021 Vladimir Ermakov. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
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

#ifndef NTPD_DRIVER__NTPDSHMDRIVER_HPP_
#define NTPD_DRIVER__NTPDSHMDRIVER_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

namespace ntpd_driver
{

/** the definition of shmTime is from ntpd source ntpd/refclock_shm.c */
struct shmTime
{
  int mode;    // 0 - if valid set
               //       use values,
               //       clear valid
               // 1 - if valid set
               //       if count before and after read of values is equal,
               //         use values
               //       clear valid
               //
  volatile int count;
  time_t clockTimeStampSec;
  int clockTimeStampUSec;
  time_t receiveTimeStampSec;
  int receiveTimeStampUSec;
  int leap;
  int precision;
  int nsamples;
  volatile int valid;
  unsigned clockTimeStampNSec;      // Unsigned ns timestamps
  unsigned receiveTimeStampNSec;    // Unsigned ns timestamps
  int dummy[8];
};

constexpr uint32_t NTPD_SHM_BASE = 0x4e545030;

using ShmTimeT = volatile struct shmTime;

class NtpdShmDriver : public rclcpp::Node
{
public:
  explicit NtpdShmDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_sub_;

  rclcpp::Parameter shm_unit_;
  rclcpp::Parameter fixup_date_;

  std::unique_ptr<ShmTimeT, std::function<void(ShmTimeT *)>> shm_;

  void time_ref_cb(const sensor_msgs::msg::TimeReference::SharedPtr msg);
  void set_system_time(const double seconds);

  ShmTimeT * attach_shmTime(int unit);
  void detach_shmTime(ShmTimeT * shm);
};

}  // namespace ntpd_driver

#endif  // NTPD_DRIVER__NTPDSHMDRIVER_HPP_
