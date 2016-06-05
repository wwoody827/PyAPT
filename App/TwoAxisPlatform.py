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