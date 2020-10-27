class PID:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

        self.period = .02
        self.inputRange = 0
        self.setpoint = 0
        self.maxInput = 0
        self.minInput = 0
        self.prevError = 0
        self.totError = 0
        self.maxInt = 1
        self.minInt = -1
        self.cont = 0
        self.posError = 0
        self.velError = 0
        self.posTolerance = 0.05
        self.velTolerance = float('inf')

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def setPID(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

    def setP(self, P):
        self.P = P
    
    def setI(self, I):
        self.I = I
    
    def setD(self, D):
        self.D = D

    def getP(self):
        return self.P

    def getI(self):
        return self.I

    def getD(self):
        return self.D

    def setPeriod(self, period):
        self.period = period

    def getPeriod(self):
        return self.period

    def setSetpoint(self, setpoint):
        if self.maxInput > self.minInput:
            self.setpoint = self.clamp(setpoint, self.minInput, self.maxInput)
        else:
            self.setpoint = setpoint
        
    def getSetpoint(self):
        return self.setpoint

    def atSetpoint(self):
        return abs(self.posError) < self.posTolerance and abs(self.velError) < self.velTolerance

    def enableContInput(self, minInput, maxInput):
        self.cont = True
        self.setInputRange(minInput, maxInput)

    def disableContInput(self):
        self.cont = False

    def setIntRange(self, minInt, maxInt):
        self.minInt = minInt
        self.maxInt = maxInt

    def setTolerance(self, posTolerance, velTolerance):
        self.posTolerance = posTolerance
        self.velTolerance = velTolerance

    def getPosError(self):
        return self.getContError(self.posError)

    def getVelError(self):
        return self.getContError(self.velError)

    def calculate(self, mes, setpoint):
        self.setSetpoint(setpoint)

        self.prevError = self.posError
        self.posError = self.getContError(self.setpoint - mes)
        self.velError = (self.posError - self.prevError) / self.period

        if self.I != 0 :
            self.totError = self.clamp(self.totError + self.posError * self.period, self.minInt / self.I, self.maxInt / self.I)

        return self.P * self.posError + self.I * self.totError + self.D * self.velError

    def reset(self):
        self.prevError = 0
        self.totError = 0

    def getContError(self, error):
        if self.cont and self.inputRange > 0:
            error %= self.inputRange

            if abs(error) > self.inputRange / 2:
                if error > 0:
                    return error - self.inputRange
                else:
                    return error + self.inputRange

        else:
            return error

    def setInputRange(self, minInput, maxInput):
        self.minInput = minInput
        self.maxInput = maxInput
        self.inputRange = maxInput - minInput

        if self.maxInput > self.minInput:
            self.setpoint = clamp(self.setpoint, self.minInput, self.maxInput)