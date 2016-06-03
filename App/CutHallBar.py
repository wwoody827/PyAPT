# -*- coding: utf-8 -*-
import time
import operator
import matplotlib.pyplot as plt
plt.ion()
import numpy as np


class TwoAxisPlatform:

    def __init__(self, max=[25, 25], stepSize=0.01):
        self.position = [0, 0]
        self.target = [0, 0]
        self.max = max
        self.stepSize = stepSize

    def setTarget(self, targetPos):
        self.target = targetPos

    def inPostion(self):
        return np.linalg.norm((np.asarray(self.position) - np.asarray(self.target))) < 0.1 * self.stepSize

    def resetPos(self, position=[0, 0]):
        self.position = position

    def getCurrentPos(self):
        return self.position

    def nextStep(self):
        nextPosList = self.target
        currentPos = np.asarray(self.getCurrentPos())

        nextPos = np.asarray(nextPosList)
        deltaPos = nextPos - currentPos
        deltaAbsPos = np.absolute(deltaPos)

        # calculate next step
        # amplitude of
        normDeltaNext = np.linalg.norm(deltaPos)

        if np.amin(deltaAbsPos) < self.stepSize:
            return nextPos.tolist()
        else:
            deltaNextMove = deltaPos / np.amin(deltaAbsPos) * self.stepSize
            deltaNextMove = deltaNextMove + currentPos
            return deltaNextMove.tolist()

########## Initialize motors #############

import sys
sys.path.insert(0, '../Driver')
from PyAPT import APTMotor
# Create object corresponding to the motor.83840805
MotorX = APTMotor(83829690, HWTYPE=31) # The number should correspond to the serial number.
MotorY = APTMotor(83840805, HWTYPE=31) # The number should correspond to the serial number.
# Use help APTMotor to obtain full list of hardware (HW) supported.

# Homing Two Motors

print("Homing X")
MotorX.go_home()
print("Homing Y")
MotorY.go_home()
print("Homing End")

origin = [10, 10]
MotorX.mAbs(origin[0])
MotorY.mAbs(origin[1])

# Note: You can control multiple motors by creating more APTMotor Objects

######### Reading Hallbar shape ##########

print("Reading Cutting Path")
with open('sixterminal 250um.txt') as f:
    polyShapeList = []
    for line in f:
        line = line.split()  # to deal with blank
        if line:            # lines (ie skip them)
            line = [float(i) for i in line]
            line = line[0:2]
            # print(line)
            polyShapeList.append(line)

polyShape = np.asarray(polyShapeList)
plt.fig = plt.plot(polyShape[:, 0], polyShape[:, 1])
plt.draw()
plt.pause(0.001)

#
platform = TwoAxisPlatform()
pathList = []
for point in polyShape:
    platform.setTarget(point)  # Set next target positino, read from Hallbar
    while(~platform.inPostion()):  # Move to target
        nextMove = platform.nextStep()
        platform.resetPos(nextMove)
        pathList.append(nextMove)
        plt.plot(nextMove[0], nextMove[1], 'ro')
        plt.draw()
        plt.pause(0.001)
        MotorX.mAbs(nextMove[0] + origin[0])
        MotorY.mAbs(nextMove[1] + origin[1])


pathArray = np.asarray(pathList)
plt.pause(1)
