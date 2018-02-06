class PIDValues:
    def __init__(self, p=0, i=0, d=0, f=0, ramprate=0, izone=0, setpoint=0, cruisevelocity=0, acceleration=0):
        self.p = p
        self.i = i
        self.d = d
        self.f = f
        self.ramprate = ramprate
        self.izone = izone
        self.setpoint = setpoint
        self.cruisevelocity = cruisevelocity
        self.acceleration = acceleration

    def __str__(self):
        retVal = "P: " + str(self.p) + "\r\n"
        retVal += "I: " + str(self.i) + "\r\n"
        retVal += "D: " + str(self.d) + "\r\n"
        retVal += "F: " + str(self.f) + "\r\n"
        retVal = "Ramp Rate: " + str(self.ramprate) + "\r\n"
        retVal += "IZone: " + str(self.izone) + "\r\n"
        retVal += "Setpoint: " + str(self.setpoint) + "\r\n"
        retVal += "Cruise Velocity: " + str(self.cruisevelocity) + "\r\n"
        retVal += "Acceleration: " + str(self.acceleration) + "\r\n"
        return retVal
