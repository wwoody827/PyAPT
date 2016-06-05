# -*- coding: utf-8 -*-
import time
import operator
import matplotlib.pyplot as plt
plt.ion()
import numpy as np
import pyclipper


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


def outline(polyShape, offset=0.01):
    numPoints = polyShape.shape[0] - 1
    offsetPolyShape = []
    for n in range(numPoints):
        # print(n)
        points = polyShape[n, :]
        prevPoint = polyShape[(n - 1) % numPoints, :]
        nextPoint = polyShape[(n + 1) % numPoints, :]

        points = np.vstack((prevPoint, points, nextPoint))
        v1x = points[1, 0] - points[0, 0]
        v1y = points[1, 1] - points[0, 1]
        v2x = points[2, 0] - points[1, 0]
        v2y = points[2, 1] - points[1, 1]
        leftTurn = v1x * v2y - v1y * v2x > 0.0

        v1 = points[1, :] - points[0, :]
        v2 = points[2, :] - points[1, :]
        newPoint = points[1, :]
        if leftTurn:
            newPoint = offset * (v1 / np.linalg.norm(v1) -
                                 v2 / np.linalg.norm(v2))
        else:
            newPoint = - offset * \
                (v1 / np.linalg.norm(v1) - v2 / np.linalg.norm(v2))
        newPoint = points[1, :] + newPoint
        offsetPolyShape.append(newPoint.tolist())

    offsetPolyShape.append(offsetPolyShape[0])
    return np.asarray(offsetPolyShape)


def createPath(polyShape, offset=0.001, loop=2):
    path = polyShape
    nextPoly = polyShape
    for n in range(loop):
        print n
        nextPoly = outline(nextPoly, offset)
        path = np.vstack((path, nextPoly))
    return path


########## Initialize motors #############

# import sys
# sys.path.insert(0, '../Driver')
# from PyAPT import APTMotor
# # Create object corresponding to the motor.83840805
# MotorX = APTMotor(83829690, HWTYPE=31) # The number should correspond to the serial number.
# MotorY = APTMotor(83840805, HWTYPE=31) # The number should correspond to the serial number.
# # Use help APTMotor to obtain full list of hardware (HW) supported.

# # Homing Two Motors

# print("Homing X")
# MotorX.go_home()
# print("Homing Y")
# MotorY.go_home()
# print("Homing End")

# origin = [10, 10]
# MotorX.mAbs(origin[0])
# MotorY.mAbs(origin[1])

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
polyShapeList.append(polyShapeList[0])
polyShape = np.asarray(polyShapeList)

######### Read Hallbar as np array #########



######### Generate Path, with loop #########
path = createPath(polyShape, offset=0.005, loop=5)
print(path.shape)
plt.fig = plt.plot(polyShape[:, 0], polyShape[:, 1])
plt.plot(path[:, 0], path[:, 1], 'r')

plt.plot(polyShape[:, 0], polyShape[:, 1], 'b')

plt.pause(1)

#
platform = TwoAxisPlatform()
pathList = path.tolist()
pathTraveledX = []
pathTraveledY = []
for point in pathList:
    platform.setTarget(point)  # Set next target positino, read from Hallbar
    while(~platform.inPostion()):  # Move to target
        nextMove = platform.nextStep()
        print(nextMove)
        platform.resetPos(nextMove)
        pathTraveledX.append(nextMove[0])
        pathTraveledY.append(nextMove[1])
        
        plt.plot(pathTraveledX, pathTraveledY, 'y')
        plt.draw()
        plt.pause(0.001)

        # MotorX.mAbs(nextMove[0] + origin[0])
        # MotorY.mAbs(nextMove[1] + origin[1])

plt.pause(1)
