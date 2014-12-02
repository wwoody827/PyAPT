# -*- coding: utf-8 -*-
"""
APT Motor Controller for Thorlabs
Adopted from
https://github.com/HaeffnerLab/Haeffner-Lab-LabRAD-Tools/blob/master/cdllservers/APTMotor/APTMotorServer.py
With thanks to SeanTanner@ThorLabs for providing APT.dll ad APT.lib


V1.0
20141125 V1.0    First working version
20141201 V1.0a   Use short notation for moving (movRelative -> mRel)

Michael Leung
mcleung@stanford.edu
"""
from ctypes import c_long, c_buffer, c_float, windll, pointer


class APTMotor():
    def __init__(self, SerialNum, HWTYPE=31):
        '''
        HWTYPE_BSC001		11	// 1 Ch benchtop stepper driver
        HWTYPE_BSC101		12	// 1 Ch benchtop stepper driver
        HWTYPE_BSC002		13	// 2 Ch benchtop stepper driver
        HWTYPE_BDC101		14	// 1 Ch benchtop DC servo driver
        HWTYPE_SCC001		21	// 1 Ch stepper driver card (used within BSC102,103 units)
        HWTYPE_DCC001		22	// 1 Ch DC servo driver card (used within BDC102,103 units)
        HWTYPE_ODC001		24	// 1 Ch DC servo driver cube
        HWTYPE_OST001		25	// 1 Ch stepper driver cube
        HWTYPE_MST601		26	// 2 Ch modular stepper driver module
        HWTYPE_TST001		29	// 1 Ch Stepper driver T-Cube
        HWTYPE_TDC001		31	// 1 Ch DC servo driver T-Cube
        HWTYPE_LTSXXX		42	// LTS300/LTS150 Long Travel Integrated Driver/Stages
        HWTYPE_L490MZ		43	// L490MZ Integrated Driver/Labjack
        HWTYPE_BBD10X		44	// 1/2/3 Ch benchtop brushless DC servo driver
        '''
        self.aptdll = windll.LoadLibrary("APT.dll")
        self.aptdll.EnableEventDlg(True)
        self.aptdll.APTInit()
        #print 'APT initialized'
        self.HWType = c_long(HWTYPE)
        self.SerialNum = c_long(SerialNum)
        self.initializeHardwareDevice()
        # TODO : Error reporting to know if initialisation went sucessfully or not.

    def getNumberOfHardwareUnits(self):
        '''
        Returns the number of HW units connected that are available to be interfaced
        '''
        numUnits = c_long()
        self.aptdll.GetNumHWUnitsEx(self.HWType, pointer(numUnits))
        return numUnits.value

    def getSerialNumber(self, index):
        '''
        Returns the Serial Number of the specified index
        '''
        HWSerialNum = c_long()
        hardwareIndex = c_long(index)
        self.aptdll.GetHWSerialNumEx(self.HWType, hardwareIndex, pointer(HWSerialNum))
        return HWSerialNum

    def initializeHardwareDevice(self):
        '''
        Initialises the motor.
        You can only get the position of the motor and move the motor after it has been initialised.
        Once initiallised, it will not respond to other objects trying to control it, until released.
        '''
        self.aptdll.InitHWDevice(self.SerialNum)
        # need some kind of error reporting here
        return True

        ''' Interfacing with the motor settings '''
    def getHardwareInformation(self):
        model = c_buffer(255)
        softwareVersion = c_buffer(255)
        hardwareNotes = c_buffer(255)
        self.aptdll.GetHWInfo(self.SerialNum, model, 255, softwareVersion, 255, hardwareNotes, 255)
        hwinfo = [model.value, softwareVersion.value, hardwareNotes.value]
        return hwinfo

    def getStageAxisInformation(self):
        minimumPosition = c_float()
        maximumPosition = c_float()
        units = c_long()
        pitch = c_float()
        self.aptdll.MOT_GetStageAxisInfo(self.SerialNum, pointer(minimumPosition), pointer(maximumPosition), pointer(units), pointer(pitch))
        stageAxisInformation = [minimumPosition.value, maximumPosition.value, units.value, pitch.value]
        return stageAxisInformation

    def setStageAxisInformation(self, minimumPosition, maximumPosition):
        minimumPosition = c_float(minimumPosition)
        maximumPosition = c_float(maximumPosition)
        units = c_long(1) #units of mm
        # Get different pitches of lead screw for moving stages for different stages.
        pitch = c_float(self.config.get_pitch())
        self.aptdll.MOT_SetStageAxisInfo(self.SerialNum, minimumPosition, maximumPosition, units, pitch)
        return True

    def getHardwareLimitSwitches(self):
        reverseLimitSwitch = c_long()
        forwardLimitSwitch = c_long()
        self.aptdll.MOT_GetHWLimSwitches(self.SerialNum, pointer(reverseLimitSwitch), pointer(forwardLimitSwitch))
        hardwareLimitSwitches = [reverseLimitSwitch.value, forwardLimitSwitch.value]
        return hardwareLimitSwitches

    def getVelocityParameters(self):
        minimumVelocity = c_float()
        acceleration = c_float()
        maximumVelocity = c_float()
        self.aptdll.MOT_GetVelParams(self.SerialNum, pointer(minimumVelocity), pointer(acceleration), pointer(maximumVelocity))
        velocityParameters = [minimumVelocity.value, acceleration.value, maximumVelocity.value]
        return velocityParameters

    def setVelocityParameters(self, minVel, acc, maxVel):
        minimumVelocity = c_float(minVel)
        acceleration = c_float(acc)
        maximumVelocity = c_float(maxVel)
        self.aptdll.MOT_SetVelParams(self.SerialNum, minimumVelocity, acceleration, maximumVelocity)
        return True

    def getVelocityParameterLimits(self):
        maximumAcceleration = c_float()
        maximumVelocity = c_float()
        self.aptdll.MOT_GetVelParamLimits(self.SerialNum, pointer(maximumAcceleration), pointer(maximumVelocity))
        velocityParameterLimits = [maximumAcceleration.value, maximumVelocity.value]
        return velocityParameterLimits

        '''
        Controlling the motors
        m = move
        c = controlled velocity
        b = backlash correction

        Rel = relative distance from current position.
        Abs = absolute position
        '''
    def getPos(self):
        '''
        Obtain the current absolute position of the stage
        '''
        position = c_float()
        self.aptdll.MOT_GetPosition(self.SerialNum, pointer(position))
        return position.value

    def mRel(self, relDistance):
        '''
        Moves the motor a relative distance specified
        relDistance    float     Relative position desired
        '''
        relativeDistance = c_float(relDistance)
        self.aptdll.MOT_MoveRelativeEx(self.SerialNum, relativeDistance, True)
        return True

    def mAbs(self, absPosition):
        '''
        Moves the motor to the Absolute position specified
        absPosition    float     Position desired
        '''
        absolutePosition = c_float(absPosition)
        self.aptdll.MOT_MoveAbsoluteEx(self.SerialNum, absolutePosition, True)
        return True

    def mcRel(self, relDistance, moveVel=0.5):
        '''
        Moves the motor a relative distance specified at a controlled velocity
        relDistance    float     Relative position desired
        moveVel        float     Motor velocity, mm/sec
        '''
        # Save velocities to reset after move
        minVel, acc, maxVel = self.getVelocityParameters()
        # Set new desired max velocity
        self.setVelocityParameters(minVel,acc,moveVel)
        self.mRel(relDistance)
        self.setVelocityParameters(minVel,acc,maxVel)
        return True

    def mcAbs(self, absPosition, moveVel=0.5):
        '''
        Moves the motor to the Absolute position specified at a controlled velocity
        absPosition    float     Position desired
        moveVel        float     Motor velocity, mm/sec
        '''
        # Save velocities to reset after move
        minVel, acc, maxVel = self.getVelocityParameters()
        # Set new desired max velocity
        self.setVelocityParameters(minVel,acc,moveVel)
        self.mAbs(absPosition)
        self.setVelocityParameters(minVel,acc,maxVel)
        return True

        ''' Miscelaneous '''
    def identify(self):
        '''
        Causes the motor to blink the Active LED
        '''
        self.aptdll.MOT_Identify(self.SerialNum)
        return True

    def cleanUpAPT(self):
        '''
        Releases the APT object
        Use when exiting the program
        '''
        self.aptdll.APTCleanUp()
        print 'APT cleaned up'
