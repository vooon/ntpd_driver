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
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/TimeReference.h>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <signal.h>
#include <cstring>
#include <cerrno>

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

static void sig_handler(int sig)
{
  put_shmTime(&g_shm);
  ros::shutdown();
}

static void time_ref_cb(const sensor_msgs::TimeReference::ConstPtr &time_ref)
{
  if (g_shm == NULL) {
    ROS_FATAL("Got time_ref before shm opens.");
    ros::shutdown();
  }

  /* header */
  g_shm->mode = 1;
  g_shm->leap = 0;        // LEAP_NOWARNING
  g_shm->precision = -1;  // initially 0.5 sec
  g_shm->nsamples = 3;    // stages of median filter

  /* ntpshm.c recommends add barrier before inc count */
  g_shm->valid = 0;
  g_shm->count += 1;
  g_shm->receiveTimeStampSec = time_ref->header.stamp.sec;
  g_shm->receiveTimeStampUSec = time_ref->header.stamp.nsec / 1000;
  g_shm->receiveTimeStampNSec = time_ref->header.stamp.nsec;
  g_shm->clockTimeStampSec = time_ref->time_ref.sec;
  g_shm->clockTimeStampUSec = time_ref->time_ref.nsec / 1000;
  g_shm->clockTimeStampNSec = time_ref->time_ref.nsec;
  /* barrier again */
  g_shm->count += 1;
  g_shm->valid = 1;

  ROS_DEBUG_THROTTLE(10, "Got time_ref: %lu.%09lu", 
      (long unsigned) time_ref->time_ref.sec, 
      (long unsigned) time_ref->time_ref.nsec);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ntpd_shm");
  ros::NodeHandle nh("~");
  ros::Subscriber time_ref_sub;

  int shm_unit;
  std::string time_ref;

  // Override default ROS handler
  signal(SIGINT, sig_handler);

  // Read Parameters
  nh.param("shm_unit", shm_unit, 2);
  nh.param<std::string>("time_ref", time_ref, "time_ref");

  g_shm = get_shmTime(shm_unit);
  if (g_shm == NULL)
    return 1;

  time_ref_sub = nh.subscribe(time_ref, 10, time_ref_cb);

  ros::spin();
  put_shmTime(&g_shm);
  return 0;
}
