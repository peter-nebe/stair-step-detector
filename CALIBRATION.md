# Calibration
This is a quick guide to calibrating the **stair-step-detector**.

Calibration is required so that the detector can output the measurement result in external world coordinates. It is also a prerequisite for the first step of detection - the determination of flat surfaces parallel to the ground plane.

Calibration fixes the world coordinate system relative to the camera. Therefore, calibration must be carried out after every change in camera mounting.

For calibration, three points are required whose positions are known in world coordinates. They should be as far apart as possible, similar to what you see in the screenshot. Each of them must be in its quarter of the measurement window. It is recommended to make them from reflector foil.

![calibration](screenshots/calibration.jpg "calibration")

The world coordinate system must be a three-dimensional right-handed Cartesian coordinate system. The x-y plane must be parallel to the ground plane and the z-axis must point vertically upwards. You can set the coordinate origin as convenient in your case.

Enter the world coordinates of the points in the [calibration-triangle](https://github.com/peter-nebe/stair-step-detector/blob/master/calibration-triangle) file in the order top-left, top-right, bottom. The unit is meters. Example:
```
calibration triangle
x1 = -1.121, y1 = 1.79826, z1 = 0.004
x2 = 1.121, y2 = 1.79826, z2 = 0.004
x3 = 0.769229, y3 = 0.3, z3 = 0.004
lowerQuadrant = right
```
### Alternative creation of the *calibration-triangle* file
It may be difficult to determine the world coordinates of your calibration points. In this case, you can arbitrarily set the coordinate origin and direction of the x-axis. For example, simply place the y-axis on the perpendicular bisector of the line connecting the first two points. All you have to do is measure the distances between the three points and enter them as side lengths a, b, c in a file called *triangle-parameters*, again in meters. You can set y3 arbitrarily. z is the height. Example:
```
triangle parameters
a = 2.242
b = 2.412
c = 1.539
y3 = 0.3
z = 0.004
```
Compile and run the [calculate-triangle](https://github.com/peter-nebe/stair-step-detector/blob/master/calculate-triangle.cpp) utility. It will read *triangle-parameters* and calculate and save the *calibration-triangle*.

### The calibration
The [calibrate](https://github.com/peter-nebe/stair-step-detector/blob/master/calibrate.cpp) program is built by [CMakeLists.txt](https://github.com/peter-nebe/stair-step-detector/blob/master/CMakeLists.txt#L58). Running the program will show the *calibration* window (see screenshot above). It will take some measurements of the calibration points and save the result in a *calibration-points* file.
