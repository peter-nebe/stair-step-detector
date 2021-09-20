import sys, json

def clear(n = 2):
  print("\033[%dJ" % n)

def move(row, col):
  print("\033[%d;%dH" % (row, col), end='')

def posPrint(row, col, number):
  move(row, col)
  print(f'{number:6.3f}', end='')

def printPoint(row, col, p):
  posPrint(row, col, p[0])
  posPrint(row + 1, col, p[1])

def printStairStep(top, left, stairStep):
  bottom = top + 3
  right = left + 20

  height = stairStep[0][1]
  posPrint(top + 2, left + 10, height)

  quadri = stairStep[1]
  printPoint(top, left, quadri[3])
  printPoint(top, right, quadri[4])
  printPoint(bottom, left, quadri[1])
  printPoint(bottom, right, quadri[2])
  printPartingLine(bottom + 2, left)

def printPartingLine(row, col):
  move(row, col)
  print("__________________________")

clear()
while line := sys.stdin.readline():
  jdata = json.loads(line)
  ident = jdata[0]
  numStairSteps = jdata[1][1]

  currentRow = 1
  left = 1
  move(currentRow, left)
  print(ident, ", number of stair steps: ", numStairSteps, sep='')

  if numStairSteps > 0:
    currentRow += 1
    left += 2
    printPartingLine(currentRow, left)
    currentRow += 2

    stairSteps = jdata[2]
    for i in range(numStairSteps - 1, -1, -1):
      printStairStep(currentRow, left, stairSteps[i])
      currentRow += 7

  else:
    clear(0)

print()