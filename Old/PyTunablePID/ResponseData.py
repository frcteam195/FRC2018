class ResponseData:
    def __init__(self, name="", desiredValue=0, actualValue=0, integralAccum=0):
        self.name = name
        self.desiredValue = desiredValue
        self.actualValue = actualValue
        self.integralAccum = integralAccum

    def __str__(self):
        retVal = "Name: " + str(self.name) + "\r\n"
        retVal += "\tDesired Value: " + str(self.desiredValue) + "\r\n"
        retVal += "\tActual Value: " + str(self.actualValue) + "\r\n"
        retVal += "\tIntegral Value: " + str(self.integralAccum) + "\r\n"
        return retVal
