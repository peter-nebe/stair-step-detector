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

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include "camera.h"

namespace stairs
{

class Window;
class GeometricTransformation;

class Pointcloud
{
public:
  Pointcloud(const Window &window, const GeometricTransformation &trans);
  void process(const Camera::DepthFrame &frame) const;

private:
  const Window &_window;
  const GeometricTransformation &_transformation;
  const rs2::pointcloud _pointcloud;
};

} /* namespace stairs */

#endif /* POINTCLOUD_H_ */
