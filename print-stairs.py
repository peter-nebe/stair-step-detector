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

import sys, json

def clear(n=2):
  print("\033[%dJ" % n)

def move(row, col):
  print("\033[%d;%dH" % (row, col), end='')

def print_number(row, col, number):
  move(row, col)
  print(f'{number:6.3f}', end='')

def print_point(row, col, point):
  print_number(row, col, point[0])
  print_number(row+1, col, point[1])

def print_stair_step(top, left, stair_step):
  height = stair_step[0][1]
  print_number(top+2, left+10, height)
  bottom = top+3
  right = left+20
  quadri = stair_step[1]
  print_point(top, left, quadri[3])
  print_point(top, right, quadri[4])
  print_point(bottom, left, quadri[1])
  print_point(bottom, right, quadri[2])
  print_parting_line(bottom+2, left)

def print_parting_line(row, col):
  move(row, col)
  print("__________________________")

clear()
while line := sys.stdin.readline():
  jdata = json.loads(line)
  ident = jdata[0]
  num_stair_steps = jdata[1][1]

  current_row, left = 1, 1
  move(current_row, left)
  print(ident, ", number of stair steps: ", num_stair_steps, sep='')

  if num_stair_steps > 0:
    current_row += 1
    left += 2
    print_parting_line(current_row, left)
    current_row += 2

    stair_steps = jdata[2]
    for i in range(num_stair_steps-1, -1, -1):
      print_stair_step(current_row, left, stair_steps[i])
      current_row += 7

  else:
    clear(0)

print()
