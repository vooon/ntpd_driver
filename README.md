ntpd\_driver
============

This ROS node listen `sensor_msgs/TimeReference` and send it to ntpd via SHM (like gpsd).

Parameter `~/shm_unit` define SHM unit (in ntp.conf) (int, default: 2).

Subscribe to topic `~/time_ref`.



System configuration
--------------------

### ntpd configuration

Add this to `/etc/ntp.conf`:

    ### GPS SHM driver
    server 127.127.28.2 minpoll 4 maxpoll 4
    fudge 127.127.28.2 time1 0.5 stratum 12 refid ROS

And then restart ntp service.

Run example:

    rosrun ntpd_driver shm_driver _shm_unit:=2 /ntpd_shm/time_ref:=/mavros/time_reference

### chrony configuration

Add this to `/etc/chrony/chrony.conf`:

    ### SHM driver
    refclock SHM 0 delay 0.5 refid ROS stratum 12

And then restart chrony service.

Run example:

    rosrun ntpd_driver shm_driver _shm_unit:=0 /ntpd_shm/time_ref:=/mavros/time_reference

