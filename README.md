Library dependencies:

* OpenCV
* Eigen3

Tested under ROS Indigo + Ubuntu 14.04 (but previous versions of both should be ok).

How to install:

* clone the repository in your `catkin_ws/src`
* run: `catkin_make`

Usage: rosrun thin_scan_matcher thin_scan_matcher_node 

Possible arguments:

```
_laser_topic:=/ros_laser_scan_topic   
_published_odom_topic:=/ros_topic_for_published_odometry_based_on_scan_matching   
_frame_skip:=#n (take a laser scan every #n)   
_bpr:=#p (#p value should be between 0 and 1 and represents the limit percentage of points which are not correctly transformed for keeping the local map generated)
```

## License

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
