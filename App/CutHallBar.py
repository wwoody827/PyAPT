# -*- coding: utf-8 -*-
import sys
sys.path.insert(0, '..\Driver')

from PyAPT import APTMotor
import time

import matplotlib.pyplot as plt
# plt.ion()
import numpy as np


# Create object corresponding to the motor.83840805
# MotorX = APTMotor(83829690, HWTYPE=31) # The number should correspond to the serial number.
# MotorY = APTMotor(83840805, HWTYPE=31) # The number should correspond to the serial number.
# Use help APTMotor to obtain full list of hardware (HW) supported.

# Homing Two Motors
#print("Homing X")
#MotorX.go_home()
#print("Homing Y")
#MotorY.go_home()
#print("Homing End")

# 
# MotorX.mAbs(10.0)
# MotorY.mAbs(10.0)
# Note: You can control multiple motors by creating more APTMotor Objects

print("Reading Cutting Path")
with open('sixterminal 250um.txt') as f:
    polyShapeList = []
    for line in f:
        line = line.split() # to deal with blank 
        if line:            # lines (ie skip them)
            line = [float(i) for i in line]
            # line = line[0,1]
            # print(line)
            polyShapeList.append(line)
          
polyShape = np.asarray(polyShapeList)
plt.plot(polyShape[:, 0], polyShape[:,1])
# plt.draw()
# plt.show()

platform = TwoAxisPlatform()
print(platform.getCurrentPos())
    
class TwoAxisPlatform:
    
    def __init__(self, max = [25, 25], stepSize = 0.1):
        self.position = [0, 0]
        self.max = max
        self.stepSize = stepSize
        
    def resetPos(self, position = [0, 0]):
        self.position = position
        
    def getCurrentPosX(self):
        return self.position
    

    def moveTo(self, nextX, nextY):
        return true