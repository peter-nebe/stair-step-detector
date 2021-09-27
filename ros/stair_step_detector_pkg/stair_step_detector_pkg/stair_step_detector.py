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

import rclpy, subprocess, json
from rclpy.node import Node
from stairs_msg.msg import Stairs, StairStep


class Publisher(Node):
  def __init__(self):
    super().__init__('stairs_publisher')
    self.publisher_ = self.create_publisher(Stairs, 'stairs_topic', 10)

  def run_detector(self):
    proc = subprocess.Popen('./detect-stairs', stdout=subprocess.PIPE, cwd='..')
    for line in proc.stdout:
      jdata = json.loads(line)
      ident = jdata[0]
      num_stair_steps = jdata[1][1]

      if ident == 'stairs' and num_stair_steps > 0:
        stair_steps = jdata[2]
        msg = Stairs()
        for i in range(num_stair_steps):
          append(stair_steps[i], msg)

        self.publisher_.publish(msg)


def append(stair_step, msg):
  msg_sstep = StairStep()
  msg_sstep.height = stair_step[0][1]
  assign(msg_sstep.quadrilateral, stair_step[1])
  msg.stair_steps.append(msg_sstep)

def assign(msg_quadri, quadri):
  to_msg(msg_quadri[0], quadri[1])
  to_msg(msg_quadri[1], quadri[2])
  to_msg(msg_quadri[2], quadri[3])
  to_msg(msg_quadri[3], quadri[4])

def to_msg(msg_point, point):
  msg_point.x = point[0]
  msg_point.y = point[1]


def main(args=None):
  rclpy.init(args=args)
  publisher = Publisher()

  publisher.run_detector()

  publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
