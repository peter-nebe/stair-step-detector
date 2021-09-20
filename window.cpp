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

#include "window.h"
#include "configuration.h"

namespace stairs
{

namespace
{

const Configuration::Streams __streams;

int calcWindowWidth()
{
  return __streams.infrared.width + __streams.depth.width;
}

int calcWindowHeight()
{
  return __streams.infrared.height;
}

inline ::rect makeRect(int x, int y, int w, int h)
{
  return{ static_cast<float>(x),
          static_cast<float>(y),
          static_cast<float>(w),
          static_cast<float>(h) };
}

} // namespace

Window::Window(const char* title)
: window(calcWindowWidth(), calcWindowHeight(), title),
  _colorMap(2) // WhiteToBlack
{
  _viewport[viewportId::infrared] = makeRect(0, 0, __streams.infrared.width, __streams.infrared.height);
  _viewport[viewportId::depth] = makeRect(__streams.infrared.width, 0, __streams.depth.width, __streams.depth.height);

  glfwSetWindowAttrib(*this, GLFW_RESIZABLE, GL_FALSE);
}

void Window::show(const Camera::Frameset &frames)
{
  auto infrared = frames.infraredFrame();
  auto depth = _colorMap.colorize(frames.depthFrame());

  window::show(infrared, _viewport[viewportId::infrared]);
  window::show(depth, _viewport[viewportId::depth]);
}

void Window::setViewport(int viewportId) const
{
  set_viewport(_viewport[viewportId]);
}

void Window::waitClose()
{
  glfwSwapBuffers(*this);

  while(!glfwWindowShouldClose(*this))
  {
    glfwWaitEvents();
    glfwSwapBuffers(*this);
  }
}

} /* namespace stairs */
