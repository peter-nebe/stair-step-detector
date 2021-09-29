#
# stair-step-detector
# Copyright (c) 2021 Peter Nebe (mail@peter-nebe.dev)
#
# This file is part of stair-step-detector.
#
# stair-step-detector is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# stair-step-detector is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with stair-step-detector.  If not, see <https://www.gnu.org/licenses/>.
#

import rclpy
from rclpy.node import Node
from stairs_msg.msg import Stairs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class Visualizer(Node):
  def __init__(self):
    super().__init__('stairs_visualizer')
    self.subscription = self.create_subscription(Stairs, 'stairs_topic', self.make_visualization, 10)
    self.publisher = self.create_publisher(Marker, 'stairs_visualizer', 10)

  def make_visualization(self, stairs):
    marker = Marker()
    self.make_treads(marker, stairs.stair_steps)
    make_vertical_faces(marker, stairs.stair_steps)
    self.publisher.publish(marker)

  def make_treads(self, mkr, stair_steps):
    mkr.header.frame_id = 'map'
    mkr.header.stamp = self.get_clock().now().to_msg()
    mkr.type = mkr.TRIANGLE_LIST
    mkr.scale.x = mkr.scale.y = mkr.scale.z = 1.0
    col = ColorRGBA(g=1.0, a=1.0)

    for step in stair_steps:
      h = step.height
      q = step.quadrilateral
      append(mkr, q[0], h, col)
      append(mkr, q[1], h, col)
      append(mkr, q[3], h, col)
      append(mkr, q[3], h, col)
      append(mkr, q[2], h, col)
      append(mkr, q[0], h, col)


def append(marker, point, _z, color):
  marker.points.append(Point(x=point.x, y=point.y, z=_z))
  marker.colors.append(color)


def make_vertical_faces(mkr, stair_steps):
  col = ColorRGBA(b=1.0, a=1.0)
  for i in range(1, len(stair_steps)):
    curr = stair_steps[i]
    pred = stair_steps[i-1]
    q = pred.quadrilateral
    append(mkr, q[2], pred.height, col)
    append(mkr, q[3], pred.height, col)
    append(mkr, q[3], curr.height, col)
    append(mkr, q[3], curr.height, col)
    append(mkr, q[2], curr.height, col)
    append(mkr, q[2], pred.height, col)


def main(args=None):
  try:
    rclpy.init(args=args)
    visualizer = Visualizer()

    rclpy.spin(visualizer)

  except KeyboardInterrupt:
    print()

  visualizer.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
