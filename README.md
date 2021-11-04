# stair-step-detector
Detection of stair steps using a LiDAR depth camera

### Dependencies
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense/releases/latest)
- [OpenCV](https://opencv.org)
- [OpenGL](https://opengl.org)
- [Boost](https://boost.org)
- [ROS](https://ros.org) (only for the ROS2 wrapper)

### Overview
![calibration](screenshots/calibration.jpg "calibration")
The camera is calibrated with the help of three points in the ground plane.

![stair-step-detector](screenshots/stair-step-detector.jpg "stair-step-detector")
After calibration, the camera can measure the heights and corner points of the stair steps that are in the measurement area. External world coordinates are displayed on the left.

Exact knowledge of the position of the individual steps can help a mobile robot to negotiate a flight of stairs, if in principle it is able to do so. For a robot, ascending and descending stairs may be different challenges. But there is only one position and line of sight for the camera to fully capture a step. If the camera is attached to the robot, it must always be brought into the correct measuring position using suitable mechanics. In principle it could look like this:

![ascending stairs](screenshots/ascending.png "ascending stairs")
![descending stairs](screenshots/descending.png "descending stairs")

If the robot is clever, it ensures that the image is not upside down by rotating the camera 180 degrees around the optical axis. If it doesn't, the image has to be rotated (or mirrored) using software. Then the algorithm of the **stair-step-detector** can be used for both ascending and descending stairs.

### Integration with ROS
If you are familiar with the [Robot Operating System](https://ros.org), you can use the [simple wrapper](https://github.com/peter-nebe/stair-step-detector/tree/master/ros) for [ROS2 Galactic Geochelone](http://docs.ros.org/en/galactic/Releases/Release-Galactic-Geochelone.html). Among other things, it contains a ROS package with which the measured stair steps can be visualized. The display is done with [RViz](http://wiki.ros.org/rviz), the 3D visualization tool for ROS. Here you can see an example:

![stairs-visualization](screenshots/stairs-visualization.png "stairs-visualization")
See [ROS Wiki](http://wiki.ros.org/stair-step-detector) for more details.

### MDE
It becomes even more convenient: If you are interested in Model-Driven Engineering (MDE), you can use the [model](https://github.com/peter-nebe/stair-step-detector/tree/master/model) of the stair-step-detector. You can combine this with other ROS components and create an ROS system without manual coding. The code generation is done by sophisticated and advanced tools.

<kbd>
  <img src="model/example-ros-system.png" />
</kbd>

*Diagram of an exemplary ROS system*

You can find the tools you need here: https://github.com/ipa320/ros-model
