 Import APTMotor class from PyAPT
from PyAPT import APTMotor
import time

# Create object corresponding to the motor.83840805
MotorX = APTMotor(83829690, HWTYPE=31) # The number should correspond to the serial number.
MotorY = APTMotor(83840805, HWTYPE=31) # The number should correspond to the serial number.
# Use help APTMotor to obtain full list of hardware (HW) supported.
MotorX.mAbs(0.0)
MotorY.mAbs(0.0)
# Note: You can control multiple motors by creating more APTMotor Objects


