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

#include <sys/ipc.h>
#include <sys/shm.h>

// there may be another library, but Poco already used by class_loader,
// so it definetly exist in your system.
#include <Poco/Format.h>
#include <Poco/Pipe.h>
#include <Poco/PipeStream.h>
#include <Poco/Process.h>
#include <Poco/StreamCopier.h>

#include <cstring>
#include <memory>
#include <string>
#include "ntpd_driver/NtpdShmDriver.hpp"


using namespace ntpd_driver; // NOLINT

/**
 * Memory barrier. Unfortunately we can't use C stdatomic.h
 * So only one option: asm magic
 *
 * from gpsd compiler.h
 */
static inline void memory_barrier(void)
{
  asm volatile (""
  :
  :
  : "memory");
}

NtpdShmDriver::NtpdShmDriver(const rclcpp::NodeOptions & options)
: Node("shm_driver", options),
  shm_unit_("shm_unit", 2),
  fixup_date_("fixup_date", false)
{
  this->declare_parameter("shm_unit", shm_unit_.get_parameter_value());
  this->declare_parameter("fixup_date", fixup_date_.get_parameter_value());

  this->get_parameter("shm_unit", shm_unit_);
  this->get_parameter("fixup_date", fixup_date_);

  // Open SHM, use Deleter to release SHM
  shm_ = std::unique_ptr<ShmTimeT, std::function<void(ShmTimeT *)>>(
    attach_shmTime(shm_unit_.as_int()),
    std::bind(&NtpdShmDriver::detach_shmTime, this, std::placeholders::_1));

  time_ref_sub_ = this->create_subscription<sensor_msgs::msg::TimeReference>(
    "time_ref", rclcpp::SensorDataQoS(),
    std::bind(&NtpdShmDriver::time_ref_cb, this, std::placeholders::_1));
}

void NtpdShmDriver::time_ref_cb(const sensor_msgs::msg::TimeReference::SharedPtr msg)
{
  auto lg = this->get_logger();
  auto clock = this->get_clock();

  const auto & time_ref = msg->time_ref;
  const auto & stamp = msg->header.stamp;

  if (!shm_) {
    RCLCPP_FATAL(lg, "Got time_ref before shm opens.");
    return;
  }

  /* header */
  shm_->mode = 1;
  shm_->nsamples = 3;    // stages of median filter

  shm_->valid = 0;
  shm_->count += 1;
  /* barrier */
  memory_barrier();
  shm_->clockTimeStampSec = time_ref.sec;
  shm_->clockTimeStampUSec = time_ref.nanosec / 1000;
  shm_->clockTimeStampNSec = time_ref.nanosec;
  shm_->receiveTimeStampSec = stamp.sec;
  shm_->receiveTimeStampUSec = stamp.nanosec / 1000;
  shm_->receiveTimeStampNSec = stamp.nanosec;
  shm_->leap = 0;          // LEAP_NOWARNING
  shm_->precision = -1;    // initially 0.5 sec
  memory_barrier();
  /* barrier again */
  shm_->count += 1;
  shm_->valid = 1;

  RCLCPP_DEBUG(
    lg, "Got time_ref: %s: %lu.%09lu",
    msg->source.c_str(),
    int64_t(time_ref.sec),
    uint32_t(time_ref.nanosec));

  /* It is a hack for rtc-less system like Raspberry Pi
   * We check that system time is unset (less than some magic value)
   * and set time.
   *
   * date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
   */
  const rclcpp::Time magic_date(1234567890ULL, 0);
  if (fixup_date_.as_bool() && clock->now() < magic_date) {
    rclcpp::Time time_ref_(time_ref);
    set_system_time(time_ref_.seconds());
  }
}

/** Sets system time by calling `sudo date`
 *
 * Sudo configuration required for that feature
 */
void NtpdShmDriver::set_system_time(const double seconds)
{
  auto lg = this->get_logger();

  RCLCPP_INFO(lg, "Setting system date to: %f", seconds);

  // construct commad: sudo -n date -u -s @1234567890.000
  Poco::Pipe outp, errp;
  Poco::Process::Args args;

  args.push_back("-n");
  args.push_back("date");
  args.push_back("-u");
  args.push_back("-s");
  args.push_back(Poco::format("@%f", seconds));

  Poco::ProcessHandle ph = Poco::Process::launch("sudo", args, 0, &outp, &errp);

  int rc = ph.wait();
  Poco::PipeInputStream outs(outp), errs(errp);
  std::string out, err;

  Poco::StreamCopier::copyToString(outs, out, 4096);
  Poco::StreamCopier::copyToString(errs, err, 4096);

  if (rc == 0) {
    RCLCPP_INFO(lg, "The system date is set.");
    RCLCPP_DEBUG_STREAM(lg, "OUT: " << out);
    RCLCPP_DEBUG_STREAM(lg, "ERR: " << err);
  } else {
    RCLCPP_ERROR(lg, "Setting system date failed.");
    RCLCPP_ERROR_STREAM(lg, "OUT: " << out);
    RCLCPP_ERROR_STREAM(lg, "ERR: " << err);
  }
}

/** Map SHM page
 *
 * NOTE: this function does not create SHM like gpsd/ntpshm.c
 */
ShmTimeT * NtpdShmDriver::attach_shmTime(int unit)
{
  auto lg = this->get_logger();

  /* we definitly not root, no IPC_CREAT */
  const int get_flags = 0666;
  key_t key = NTPD_SHM_BASE + unit;

  int shmid = shmget(key, sizeof(shmTime), get_flags);
  if (shmid < 0) {
    RCLCPP_FATAL(
      lg, "SHM(%d) shmget(0x%08lx, %zd, %o) fail: %s",
      unit, key, sizeof(shmTime), get_flags, strerror(errno));
    return nullptr;
  }

  /* TODO: check number of attached progs */

  const void * shmat_fail_ptr = (void *)-1;  // NOLINT

  void * p = shmat(shmid, 0, 0);
  if (p == shmat_fail_ptr) {
    RCLCPP_FATAL(lg, "SHM(%d) shmat(%d, 0, 0) fail: %s", unit, shmid, strerror(errno));
    return nullptr;
  }

  RCLCPP_INFO(lg, "SHM(%d) key 0x%08lx, successfully opened", unit, key);
  return static_cast<ShmTimeT *>(p);
}

/** Release SHM page
 *
 */
void NtpdShmDriver::detach_shmTime(ShmTimeT * shm)
{
  auto lg = this->get_logger();

  if (shm == nullptr) {
    return;
  }

  if (shmdt((const void *)shm) == -1) {
    RCLCPP_FATAL(lg, "SHM shmdt(%p) fail: %s", shm, strerror(errno));
  }
}


#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ntpd_driver::NtpdShmDriver)
