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

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/TimeReference.h>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <signal.h>
#include <cstring>
#include <cerrno>

// there may be another library, but Poco already used by class_loader,
// so it definetly exist in your system.
#include <Poco/Process.h>
#include <Poco/Pipe.h>
#include <Poco/PipeStream.h>
#include <Poco/Format.h>
#include <Poco/StreamCopier.h>

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

/** Map SHM page
 *
 * NOTE: this function did not create SHM like gpsd/ntpshm.c
 */
static volatile struct shmTime *get_shmTime(int unit)
{
  /* we definitly not root, no IPC_CREAT */
  const int get_flags = 0666;

  int shmid = shmget(NTPD_SHM_BASE + unit, sizeof(shmTime), get_flags);
  if (shmid < 0) {
    ROS_FATAL("SHM shmget(0x%08lx, %zd, %o) fail: %s",
        NTPD_SHM_BASE + unit,
        sizeof(shmTime),
        get_flags,
        strerror(errno));
    return NULL;
  }

  /* TODO: check number of attached progs */

  void *p = shmat(shmid, 0, 0);
  if (p == (void *) -1) {
    ROS_FATAL("SHM shmat(%d, 0, 0) fail: %s", shmid, strerror(errno));
    return NULL;
  }

  ROS_INFO("SHM(%d) key 0x%08lx, successfully opened", unit, NTPD_SHM_BASE + unit);
  return (volatile struct shmTime*) p;
}

static void put_shmTime(volatile struct shmTime **shm)
{
  ROS_ASSERT(shm != NULL);
  if (*shm == NULL)
    return;

  if (shmdt((const void*) *shm) == -1)
    ROS_FATAL("SHM shmdt(%p) fail: %s", *shm, strerror(errno));
  else
    *shm = NULL;
}


/** global SHM time handle */
volatile struct shmTime *g_shm = NULL;
static bool g_set_date = false;

static void sig_handler(int sig)
{
  put_shmTime(&g_shm);
  ros::shutdown();
}

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

static void time_ref_cb(const sensor_msgs::TimeReference::ConstPtr &time_ref)
{
  if (g_shm == NULL) {
    ROS_FATAL("Got time_ref before shm opens.");
    ros::shutdown();
  }

  /* header */
  g_shm->mode = 1;
  g_shm->nsamples = 3;    // stages of median filter

  g_shm->valid = 0;
  g_shm->count += 1;
  /* barrier */
  memory_barrier();
  g_shm->clockTimeStampSec = time_ref->time_ref.sec;
  g_shm->clockTimeStampUSec = time_ref->time_ref.nsec / 1000;
  g_shm->clockTimeStampNSec = time_ref->time_ref.nsec;
  g_shm->receiveTimeStampSec = time_ref->header.stamp.sec;
  g_shm->receiveTimeStampUSec = time_ref->header.stamp.nsec / 1000;
  g_shm->receiveTimeStampNSec = time_ref->header.stamp.nsec;
  g_shm->leap = 0;        // LEAP_NOWARNING
  g_shm->precision = -1;  // initially 0.5 sec
  memory_barrier();
  /* barrier again */
  g_shm->count += 1;
  g_shm->valid = 1;

  ROS_DEBUG_THROTTLE(10, "Got time_ref: %s: %lu.%09lu",
      time_ref->source.c_str(),
      (long unsigned) time_ref->time_ref.sec,
      (long unsigned) time_ref->time_ref.nsec);

  /* It is a hack for rtc-less system like Raspberry Pi
   * We check that system time is unset (less than some magick)
   * and set time.
   *
   * Sudo configuration required for that feature
   * date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
   */
  if (g_set_date && ros::Time::now().sec < 1234567890ULL) {

    const double stamp = time_ref->time_ref.toSec();

    ROS_INFO("Setting system date to: %f", stamp);

    // construct commad: sudo -n date -u -s @1234567890.000
    Poco::Pipe outp, errp;
    Poco::Process::Args args;
    args.push_back("-n");
    args.push_back("date");
    args.push_back("-u");
    args.push_back("-s");
    args.push_back(Poco::format("@%f", stamp));
    Poco::ProcessHandle ph = Poco::Process::launch("sudo", args, 0, &outp, &errp);

    int rc = ph.wait();
    Poco::PipeInputStream outs(outp), errs(errp);
    std::string out, err;

    Poco::StreamCopier::copyToString(outs, out, 4096);
    Poco::StreamCopier::copyToString(errs, err, 4096);

    if (rc == 0) {
      ROS_INFO("The system date is set.");
      ROS_DEBUG_STREAM("OUT: " << out);
      ROS_DEBUG_STREAM("ERR: " << err);
    }
    else {
      ROS_ERROR("Setting system date failed.");
      ROS_ERROR_STREAM("OUT: " << out);
      ROS_ERROR_STREAM("ERR: " << err);
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ntpd_shm");

  ros::NodeHandle nh("~");
  ros::Subscriber time_ref_sub;

  int shm_unit;
  std::string time_ref_topic;

  // Override default ROS handler
  signal(SIGINT, sig_handler);

  // Read Parameters
  nh.param("shm_unit", shm_unit, 2);
  nh.param("fixup_date", g_set_date, false);
  nh.param<std::string>("time_ref_topic", time_ref_topic, "time_ref");

  // Report settings
  ROS_INFO_STREAM("NTP time source: " << ros::names::resolve(time_ref_topic, true));
  ROS_INFO_STREAM("NTP date fixup: " << ((g_set_date) ? "enabled" : "disabled"));

  g_shm = get_shmTime(shm_unit);
  if (g_shm == NULL)
    return 1;

  // prefer to unreliable connection, but accept tcp too.
  time_ref_sub = nh.subscribe(time_ref_topic, 10, time_ref_cb,
      ros::TransportHints()
      .unreliable().maxDatagramSize(1024)
      .reliable().tcpNoDelay(true));

  ros::spin();
  put_shmTime(&g_shm);
  return 0;
}
