# -*- coding: utf-8 -*-
import time
import operator

import matplotlib.pyplot as plt
plt.ion()
import numpy as np
import pyclipper


import sys
sys.path.insert(0, '../Driver')
from PyAPTDummy import APTMotorD
# load dummy motors for test
MotorX = APTMotorD()
MotorY = APTMotorD()

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

import sys
sys.path.insert(0, '../Driver')


# Homing Two Motors

import argparse
parser = argparse.ArgumentParser(prog='Menu', description='description')
parser.add_argument(
    'cmd', choices=['motorinit', 'origin', 'goto', 'done', 'redo', 'home', 'quit'])

motor = argparse.ArgumentParser(prog='Motor', description='description')
motor.add_argument("-x", type=float, default=0)
motor.add_argument("-y", type=float, default=0)
print("Adjust Origin Position")

# Wait for adjust
origin = [10, 10]
while True:
    astr = raw_input('$: ')
    # print astr
    try:
        args = parser.parse_args(astr.split())
    except SystemExit:
        # trap argparse error message
        print 'error'
        continue
    if args.cmd in ['done']:
        origin = [MotorX.getPos(), MotorX.getPos()]
        break

    elif args.cmd in ['motorinit']:
        from PyAPT import APTMotor
        # Create object corresponding to the motor.83840805
        # The number should correspond to the serial number.
        MotorX = APTMotor(83829690, HWTYPE=31)
        # The number should correspond to the serial number.
        MotorY = APTMotor(83840805, HWTYPE=31)
        # Use help APTMotor to obtain full list of hardware (HW) supported.

    elif args.cmd in ['origin']:
        origin = [MotorX.getPos(), MotorX.getPos()]

    elif args.cmd in ['goto']:
        astr = raw_input('Move: ')
        args = motor.parse_args(astr.split())
        print args.x
        print args.y
        MotorX.mRel(args.x)
        MotorY.mRel(args.y)
        print("End")

    elif args.cmd in ['home']:
        print("Homing X")
        MotorX.go_home()
        print("Homing Y")
        MotorY.go_home()
        print("Homing End")
        continue
    else:
        continue


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

######### End: Read Hallbar as np array #########


######### Generate Path, with loop #########
path = createPath(polyShape, offset=0.01, loop=5)
print(path.shape)
plt.fig = plt.plot(polyShape[:, 0], polyShape[:, 1])
plt.plot(path[:, 0], path[:, 1], 'r')

plt.plot(polyShape[:, 0], polyShape[:, 1], 'b')

plt.pause(1)
pathXMax = np.amax(path[:,0])
pathXMin = np.amin(path[:,0])
pathYMax = np.amax(path[:,1])
pathYMin = np.amin(path[:,1])
boundary = np.asarray([[pathXMin, pathYMin],[pathXMin, pathYMax], [pathXMax, pathYMax], [pathXMax, pathYMin]]);
print(boundary)
plt.plot(boundary[:, 0], boundary[:, 1], 'g', linewidth=2.0)
######### Check Pattern and Run ##############
pattern = argparse.ArgumentParser(prog='Pattern', description='description')
pattern.add_argument('cmd', choices=['check', 'run', 'goto', 'quit', 'origin'])


while True:
    astr = raw_input("'check','run', 'goto', 'quit', 'origin'$: ")
    args = pattern.parse_args(astr.split())

    if args.cmd in ["run"]:
        origin = [MotorX.getPos(), MotorX.getPos()]
        platform = TwoAxisPlatform()
        pathList = path.tolist()
        pathTraveledX = []
        pathTraveledY = []
        for point in pathList:
         # Set next target positino, read from Hallbar
            platform.setTarget(point)
            while(~platform.inPostion()):  # Move to target
                nextMove = platform.nextStep()
                print(nextMove)
                platform.resetPos(nextMove)
                pathTraveledX.append(nextMove[0])
                pathTraveledY.append(nextMove[1])
                plt.plot(pathTraveledX, pathTraveledY, 'y')
                plt.draw()
                plt.pause(0.001)

                MotorX.mAbs(nextMove[0] + origin[0])
                MotorY.mAbs(nextMove[1] + origin[1])

        plt.pause(1)

    elif args.cmd in ["check"]:
        origin = [MotorX.getPos(), MotorX.getPos()]
        platform = TwoAxisPlatform()
        pathList = boundary.tolist()
        pathList.append(origin)
        pathTraveledX = []
        pathTraveledY = []
        for point in pathList:
         # Set next target positino, read from Hallbar
            platform.setTarget(point)
            while(~platform.inPostion()):  # Move to target
                nextMove = platform.nextStep()
                print(nextMove)
                platform.resetPos(nextMove)
                pathTraveledX.append(nextMove[0])
                pathTraveledY.append(nextMove[1])
                plt.plot(pathTraveledX, pathTraveledY, 'y')
                plt.draw()
                plt.pause(0.5)

                MotorX.mAbs(nextMove[0] + origin[0])
                MotorY.mAbs(nextMove[1] + origin[1])

        plt.pause(1)

    elif args.cmd in ["goto"]:
        astr = raw_input('Move: ')
        args = motor.parse_args(astr.split())
        print args.x
        print args.y
        MotorX.mRel(args.x)
        MotorY.mRel(args.y)
        print("End")

    elif args.cmd in ["origin"]:
        MotorX.mAbs(origin[0])
        MotorY.mAbs(origin[1])
        print("Start Point Reset!!")

    elif args.cmd in ["quit"]:
        break
    else:
        continue
########## End: Check for boundary ##############
