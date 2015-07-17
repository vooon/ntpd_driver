ntpd\_driver
============

This ROS node listen `sensor_msgs/TimeReference` and send it to ntpd via SHM (like gpsd).

Parameter `~/shm_unit` define SHM unit (in ntp.conf) (int, default: 2).
Parameter `~/time_ref_topic` define the topic to subscribe to (string, default: `"~/time_ref"`).
Parameter `~/fixup_date` enable/disable date fixup (bool, default: false)


System configuration
--------------------

### ntpd configuration

Add this to `/etc/ntp.conf`:

    ### GPS SHM driver
    server 127.127.28.2 minpoll 4 maxpoll 4
    fudge 127.127.28.2 time1 0.5 stratum 12 refid ROS

And then restart ntp service.

Run example:

    rosrun ntpd_driver shm_driver _shm_unit:=2 _time_ref_topic:=/mavros/time_reference


### chrony configuration

Add this to `/etc/chrony/chrony.conf`:

    ### SHM driver
    refclock SHM 0 delay 0.5 refid ROS

And then restart chrony service.

Run example:

    rosrun ntpd_driver shm_driver _shm_unit:=0 _time_ref_topic:=/mavros/time_reference


### Date fixup configuration (sudo)

On my Raspberry Pi 2 ntpd reject SHM data if system date is not set (e.g. JAN 1970).
To fix that `shm_driver` now can set system time if it unset.

For setting date program requires root privileges, so used `sudo`.

Add this to `/etc/sudoers` (using `visudo`):

    %sudo	ALL=NOPASSWD: /bin/date

