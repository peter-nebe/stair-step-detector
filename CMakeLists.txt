cmake_minimum_required (VERSION 3.17)
project (stair-step-detector)

set (realsense_DIR
  /hd/d1/realsense/librealsense-2.42.0
)
set (opencv_DIR
  /usr/include/opencv4
)
include_directories (
  ${realsense_DIR}/include
  ${realsense_DIR}/examples
  ${opencv_DIR}
)
link_libraries (
  realsense2
  opencv_imgproc
  opencv_highgui
  opencv_core
  glfw
  GL
)
set (CMAKE_CXX_STANDARD
  20
)
set (CMAKE_CXX_STANDARD_REQUIRED
  ON
)
set (detect-stairs_SRC
  calibrationMark.cpp
  calibrationTriangle.cpp
  camera.cpp
  detect-stairs.cpp
  drawing.cpp
  geometricCalibration.cpp
  pointcloud.cpp
  quadrilateralTest.cpp
  segmentation.cpp
  stairs.cpp
  transformation.cpp
  window.cpp
)
add_executable (
  detect-stairs
  ${detect-stairs_SRC}
)
set (calibrate_SRC
  calibrate.cpp
  calibrationMark.cpp
  calibrationTriangle.cpp
  camera.cpp
  drawing.cpp
  geometricCalibration.cpp
  transformation.cpp
  window.cpp
)
add_executable (
  calibrate
  ${calibrate_SRC}
)
