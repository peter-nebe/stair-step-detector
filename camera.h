/*
 * stair-step-detector
 * Copyright (c) 2021 Peter Nebe (mail@peter-nebe.dev)
 *
 * This file is part of stair-step-detector.
 *
 * stair-step-detector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * stair-step-detector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with stair-step-detector.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "types.h"
#include "librealsense2/rs.hpp"
#include "librealsense2/rsutil.h"

namespace stairs
{

class Camera
{
public:
  struct VideoFrame : rs2::video_frame
  {
    VideoFrame(const rs2::video_frame &vf) : rs2::video_frame(vf) {}
    int width() const { return get_width(); }
    int height() const { return get_height(); }
    int stride() const { return get_stride_in_bytes(); }
    int bytesPerPixel() const { return get_bytes_per_pixel(); }
    const void *pixels() const { return get_data(); }
  };

  struct DepthFrame : rs2::depth_frame
  {
    DepthFrame(const rs2::depth_frame &df) : rs2::depth_frame(df) {}
    const void *pixels() const { return get_data(); }
    float distance(int x, int y) const { return get_distance(x, y); }

    template<size_t N>
    using ProjectSrc_t = std::array<Point3, N>;
    template<size_t N>
    using ProjectDst_t = std::array<Point2f, N>;
    template<size_t N>
    void project(const ProjectSrc_t<N> &points, ProjectDst_t<N> &pixels) const;

    template<size_t N>
    using DeprojectSrc_t = std::array<Point2, N>;
    template<size_t N>
    using DeprojectDst_t = std::array<Point3f, N>;
    template<size_t N>
    void deproject(const DeprojectSrc_t<N> &pixels, DeprojectDst_t<N> &points) const;
  };

  struct Frameset : rs2::frameset
  {
    Frameset(const rs2::frame &f) : rs2::frameset(f) {}
    VideoFrame grayscaleFrame() const { return get_color_frame(); }
    VideoFrame infraredFrame() const { return get_infrared_frame(); }
    DepthFrame depthFrame() const { return get_depth_frame(); }
  };

  void start();
  Frameset waitForFrames();

private:
  rs2::pipeline _pipe;
}; // class Camera

template<size_t N>
void Camera::DepthFrame::project(const ProjectSrc_t<N> &points, ProjectDst_t<N> &pixels) const
{
  const rs2_intrinsics intrinsics = get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  auto pixel = pixels.begin();
  for(const Point3 &point : points)
  {
    const float pnt[3]{ static_cast<float>(point.x),
                        static_cast<float>(point.y),
                        static_cast<float>(point.z) };
    float pxl[2];

    rs2_project_point_to_pixel(pxl, &intrinsics, pnt);

    *pixel++ = { pxl[0], pxl[1] };
  }
}

template<size_t N>
void Camera::DepthFrame::deproject(const DeprojectSrc_t<N> &pixels, DeprojectDst_t<N> &points) const
{
  const rs2_intrinsics intrinsics = get_profile().as<rs2::video_stream_profile>().get_intrinsics();

  auto point = points.begin();
  for(const Point2 &pixel : pixels)
  {
    const float depth = distance(pixel.x + 0.5, pixel.y + 0.5);
    const float pxl[2]{ static_cast<float>(pixel.x),
                        static_cast<float>(pixel.y) };
    float pnt[3];

    rs2_deproject_pixel_to_point(pnt, &intrinsics, pxl, depth);

    *point++ = { pnt[0], pnt[1], pnt[2] };
  }
}

} /* namespace stairs */

#endif /* CAMERA_H_ */
