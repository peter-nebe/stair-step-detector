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

#ifndef WINDOW_H_
#define WINDOW_H_

#include "example.hpp"
#include "camera.h"

namespace stairs
{

namespace viewportId
{
  const int grayscale = 0;
  const int depth = 1;
  const int infrared = 2;
  const int numIds = 3;
}

class Window : public window
{
public:
  Window(const char* title);
  void show(const Camera::Frameset &frames);
  void setViewport(int viewportId) const;
  void waitClose();

private:
  rs2::colorizer _colorMap;
  ::rect _viewport[viewportId::numIds]{};
};

} /* namespace stairs */

#endif /* WINDOW_H_ */
