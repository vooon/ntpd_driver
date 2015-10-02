^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ntpd_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2015-10-02)
------------------
* `#2 <https://github.com/vooon/ntpd_driver/issues/2>`_: allow both UDPROS and TCPROS
* Contributors: Vladimir Ermakov

1.1.1 (2015-07-18)
------------------
* Syncronize shm write with gpsd/ntpshmwrite.c
* Add fixup_date parameter.
  Now driver can setup system date.
* Contributors: Vladimir Ermakov

1.1.0 (2015-04-17)
------------------
* Update package.xml for REP 140
* Change topic param name to `~/time_ref_topic`
* Merge pull request `#1 <https://github.com/vooon/ntpd_driver/issues/1>`_ from oceansystemslab/master
  Updated Driver
* Updated README with new node input parameter
* fixed indentation according to ROS guidelines and added input parameter for time reference topic
* Removed transport hints to allow the driver to work with the nmea_serial_driver Python publisher and added ROS style indentation.
* Updated .gitignore with C++ version on GitHub
* Contributors: Valerio De Carolis, Vladimir Ermakov

1.0.2 (2015-01-24)
------------------
* Add example launch script.
* Add rosindex metadata.
* Contributors: Vladimir Ermakov

1.0.1 (2014-07-13)
------------------
* Rename package from ros2ntpd to ntpd_driver
  Add notes for chrony.
* Contributors: Vladimir Ermakov

1.0.0 (2014-06-28)
------------------
* Update readme
* Add readme
* Update shmTime struct to latest version.
  Fix: change leap=0 will fix ntpd receive.
* Initial
* Contributors: Vladimir Ermakov
