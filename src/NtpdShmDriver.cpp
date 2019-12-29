
#include <ntpd_driver/NtpdShmDriver.hpp>
#include <class_loader/register_macro.hpp>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <cstring>

// there may be another library, but Poco already used by class_loader,
// so it definetly exist in your system.
#include <Poco/Process.h>
#include <Poco/Pipe.h>
#include <Poco/PipeStream.h>
#include <Poco/Format.h>
#include <Poco/StreamCopier.h>

/**
 * Memory barrier. unfortunatly we can't use C stdatomic.h
 * So only one option: asm magick
 *
 * from gpsd compiler.h
 */
static inline void memory_barrier(void)
{
  asm volatile ("" : : : "memory");
}

NtpdShmDriver::NtpdShmDriver() :
  Node("shm_driver"),
  shm_unit_(2),
  fixup_date_(false),
  time_ref_topic("time_ref")
{

  // Open SHM, use Deleter to release SHM
  shm_ = std::unique_ptr<ShmTimeT, std::function<void(ShmTimeT*)>>(
      attach_shmTime(shm_unit_),
      std::bind(&NtpdShmDriver::detach_shmTime, this, std::placeholders::_1)
      );
}

void NtpdShmDriver::time_ref_cb(const sensor_msgs::msg::TimeReference::SharedPtr msg)
{
  auto lg = this->get_logger();
  const auto &time_ref = msg->time_ref;
  const auto &stamp = msg->header.stamp;

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
  shm_->leap = 0;        // LEAP_NOWARNING
  shm_->precision = -1;  // initially 0.5 sec
  memory_barrier();
  /* barrier again */
  shm_->count += 1;
  shm_->valid = 1;

#if 0
  RCLCPP_DEBUG_THROTTLE(lg, RCUTILS_STEADY_TIME, 10, "Got time_ref: %s: %lu.%09lu",
      time_ref->source.c_str(),
      (long unsigned) time_ref->time_ref.sec,
      (long unsigned) time_ref->time_ref.nanosec);
#endif

  /* It is a hack for rtc-less system like Raspberry Pi
   * We check that system time is unset (less than some magic value)
   * and set time.
   *
   * Sudo configuration required for that feature
   * date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
   */
  if (fixup_date_ && /*ros::Time::now().sec*/0 < 1234567890ULL) {
    rclcpp::Time time_ref_(time_ref);
    set_system_time(time_ref_.seconds());
  }
}

void NtpdShmDriver::set_system_time(const double seconds)
{
  auto lg= this->get_logger();

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
  }
  else {
    RCLCPP_ERROR(lg, "Setting system date failed.");
    RCLCPP_ERROR_STREAM(lg, "OUT: " << out);
    RCLCPP_ERROR_STREAM(lg, "ERR: " << err);
  }
}

/** Map SHM page
 *
 * NOTE: this function did not create SHM like gpsd/ntpshm.c
 */
ShmTimeT* NtpdShmDriver::attach_shmTime(int unit)
{
  auto lg = this->get_logger();

  /* we definitly not root, no IPC_CREAT */
  const int get_flags = 0666;
  key_t key = NTPD_SHM_BASE + unit;

  int shmid = shmget(key, sizeof(shmTime), get_flags);
  if (shmid < 0) {
    RCLCPP_FATAL(lg, "SHM(%d) shmget(0x%08lx, %zd, %o) fail: %s",
        unit, key, sizeof(shmTime), get_flags, strerror(errno));
    return nullptr;
  }

  /* TODO: check number of attached progs */

  void *p = shmat(shmid, 0, 0);
  if (p == (void *) -1) {
    RCLCPP_FATAL(lg, "SHM(%d) shmat(%d, 0, 0) fail: %s", unit, shmid, strerror(errno));
    return nullptr;
  }

  RCLCPP_INFO(lg, "SHM(%d) key 0x%08lx, successfully opened", unit, key);
  return static_cast<ShmTimeT*>(p);
}

void NtpdShmDriver::detach_shmTime(ShmTimeT* shm)
{
  auto lg = this->get_logger();

  if (shm == nullptr)
    return;

  if (shmdt((const void*) shm) == -1) {
    RCLCPP_FATAL(lg, "SHM shmdt(%p) fail: %s", shm, strerror(errno));
  }
}

CLASS_LOADER_REGISTER_CLASS(NtpdShmDriver, rclcpp::Node)
