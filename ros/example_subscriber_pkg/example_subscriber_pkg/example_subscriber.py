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


def clear(n=2):
  print("\033[%dJ" % n)

def move(row, col):
  print("\033[%d;%dH" % (row, col), end='')

def print_number(row, col, number):
  move(row, col)
  print(f'{number:6.3f}', end='')

def print_point(row, col, point):
  print_number(row, col, point.x)
  print_number(row+1, col, point.y)

def print_stair_step(top, left, stair_step):
  print_number(top+2, left+10, stair_step.height)
  bottom = top+3
  right = left+20
  q = stair_step.quadrilateral
  print_point(top, left, q[2])
  print_point(top, right, q[3])
  print_point(bottom, left, q[0])
  print_point(bottom, right, q[1])
  print_parting_line(bottom + 2, left)

def print_parting_line(row, col):
  move(row, col)
  print("__________________________")

def print_stairs(stairs):
  num_stair_steps = len(stairs.stair_steps)
  current_row, left = 1, 1
  move(current_row, left)
  print("stairs, number of stair steps:", num_stair_steps)
  current_row += 1
  left += 2
  print_parting_line(current_row, left)
  current_row += 2
  for i in range(num_stair_steps-1, -1, -1):
    print_stair_step(current_row, left, stairs.stair_steps[i])
    current_row += 7


class Subscriber(Node):
  def __init__(self):
    super().__init__('stairs_subscriber')
    self.subscription = self.create_subscription(Stairs, 'stairs_topic', print_stairs, 10)
    clear()


def main(args=None):
  try:
    rclpy.init(args=args)
    subscriber = Subscriber()

    rclpy.spin(subscriber)

  except KeyboardInterrupt:
    print()

  subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
