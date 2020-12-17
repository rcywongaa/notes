Aruco (as implemented in OpenCV)

- pros
  - Easy to set up (with readily available [aruco marker generator](http://ev.me/arucogen/), opencv & ros implementation, etc.)
  - fewer false detection (with default parameters)
- cons
  - Newer versions of aruco is GPL licensed, hence opencv is stuck on an old implementation of aruco when it was still BSD.
  - More susceptible to [rotational ambiguity](https://docs.google.com/document/d/1QU9KoBtjSM2kF6ITOjQ76xqL7H0TEtXriJX5kwi9Kgc/edit#bookmark=id.ve2m6nbh7fg0) at medium to long ranges
  - More [tuning parameters](https://docs.opencv.org/3.4/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html)
  - More computationally intensive

AprilTag (as implemented in [apriltag_ros](http://wiki.ros.org/apriltag_ros))

- pros
  - BSD license
  - Fewer [tuning parameters](http://wiki.ros.org/apriltag_ros/Tutorials/Detection%20in%20a%20video%20stream)
  - Works fairly well even at long range
  - Used by [NASA](https://github.com/nasa/AprilNav)
  - More flexible marker design (e.g. markers not necessarily square)
  - Less computationally intensive
- cons
  - less straight forward to setup (no opencv implementation AFAIK, only ros implementation, slightly more steps to [obtain markers](https://github.com/AprilRobotics/apriltag-imgs))
  - more false detection (with default parameters)
