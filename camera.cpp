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

#include "camera.h"
#include "configuration.h"

namespace stairs
{

void Camera::start()
{
  const Configuration::Streams streams;
  rs2::config rsconf;
  rsconf.disable_all_streams();

  if(streams.grayscale.active)
    rsconf.enable_stream(RS2_STREAM_COLOR, streams.grayscale.width, streams.grayscale.height, RS2_FORMAT_Y16, 15);

  if(streams.infrared.active)
    rsconf.enable_stream(RS2_STREAM_INFRARED, streams.infrared.width, streams.infrared.height);

  if(streams.depth.active)
    rsconf.enable_stream(RS2_STREAM_DEPTH, streams.depth.width, streams.depth.height);

  rs2::pipeline_profile profile = _pipe.start(rsconf);
  profile.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_VISUAL_PRESET, RS2_L500_VISUAL_PRESET_MAX_RANGE);
}

Camera::Frameset Camera::waitForFrames()
{
  return _pipe.wait_for_frames();
}

} /* namespace stairs */
