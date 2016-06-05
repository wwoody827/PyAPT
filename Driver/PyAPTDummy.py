class APTMotorD():

    def __init__(self, SerialNum=None, HWTYPE=31, verbose=False):
        self.pos = 0
        self.v = 0
        self.verbose = verbose
        self.Connected = True


    def getSerialNumberByIdx(self, index):
        '''
        Returns the Serial Number of the specified index
        '''
        
        return 0

    def setSerialNumber(self, SerialNum):
        '''
        Sets the Serial Number of the specified index
        '''
        
        return self.SerialNum.value

    def initializeHardwareDevice(self):
        '''
        Initialises the motor.
        You can only get the position of the motor and move the motor after it has been initialised.
        Once initiallised, it will not respond to other objects trying to control it, until released.
        '''
        return True

        ''' Interfacing with the motor settings '''
    
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
        return self.pos

    def mRel(self, relDistance):
        '''
        Moves the motor a relative distance specified
        relDistance    float     Relative position desired
        '''
        self.pos = self.pos + relDistance
        return True

    def mAbs(self, absPosition):
        '''
        Moves the motor to the Absolute position specified
        absPosition    float     Position desired
        '''
        self.pos = absPosition
        return True

    
    def go_home(self):
        '''
        Move the stage to home position and reset position entry
        '''
        self.pos = 0
        return True
    
    
        ''' Miscelaneous '''
  