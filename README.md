ros2ntpd
========

This ROS node listen ROS `sensor_msgs/TimeReference` and send it to ntpd (like gpsd).

Parameter `~/shm_unit` define SHM unit (in ntp.conf) (int, default: 2).
Subscribed topic `~/time_ref` defines time source.


System configuration
--------------------

Add this to `/etc/ntp.conf`:

    ### GPS SHM driver
    server 127.127.28.2 minpoll 4 maxpoll 4
    fudge 127.127.28.2 time1 0.5 stratum 16 refid ROS

And then restart ntp service.

Run example:

    rosrun ros2ntpd ros2ntpd /ros2ntpd/time_ref:=/mavros/time_reference

