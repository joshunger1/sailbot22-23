# Replace angle with encoder positons or do conversions after the fact
prevErrors = [0, 0, 0, 0, 0]
errorSum = 0
kp = 0.5
ki = 0.01
kd = 0.1
activeKp = 0.5
#portRoll = True # need info on how to track
adc = 0.48

def checkDesiredRoll():
    # Check wind angle, then check current tilt of boat, then adjust ballast accordingly
    self.lastRollAngle.append(self.airmar_data["roll"])
    smooth_angle = self.median(self.lastWinds)
    if (0 < smooth_angle <= 180): # starboard tack
        return False
    else:
        return True # port tack
    ## self.get_logger().info("roll:" + self.airmar_data["roll"])
    # delta = self.airmar_data["roll"] - self.lastRollAngle[-1]

def ballast_alg_passive(): # this alg tells the ballast to go hard port or hard starboard depending on the tack
    # NOTE: max port ADC value for ballast is 0.16; starboard is 0.83; midship is 0.5
    portRoll # True or False depending on whether we want to lean to the left/port (true) or right/starboard (false)
    if len(self.lastWinds) == 0:
        return # failsafe if we have received no data on wind to prevent crash
    else:
        portRoll = checkDesiredRoll()
    
    adcError
    if (portRoll): # if we are leaning port
        adcError = adc - 0.16 # because the max port ADC value from the potentiometer is 0.16
    else: # if we are leaning starboard
        adcError = 0.83 - adc # because the max starboard ADC value from the potentiometer is 0.83
    
    # make more efficient with rolling overwrite # used in integral error and derivative error
    # prevErrors[4] = prevErrors[3]
    # prevErrors[3] = prevErrors[2]
    # prevErrors[2] = prevErrors[1]
    # prevErrors[1] = prevErrors[0]
    # prevErrors[0] = adcError

    if (adcError > 0.015): # if the ballast needs to move...
        #integralAngleError = sum(prevErrors)
        #derivativeAngleError = (adcError - prevErrors[1])*0.01 # Change 0.01 to avg time between changes [or just delete derivative lol]

        errorSum = adcError * kp #+ integralAngleError * ki #+ derivativeAngleError * kd

        if (adcError < 0.12): # if the ballast is close to it's goal and needs to slow down (0.12 was chosen arbitrarily)
            errorSum *= (adcError * 8.0) # this will linearly decrease the speed at which the ballast
                                         # is moving based on how close the ballast is to it's goal
        
        # translate the PID error into our output range; the motor accepts 60-130, with 60
        # being full tilt port and 130 being full tilt starboard; 95 is the median value
        ballast_speed = 95
        if (portRoll): # if the motor needs to send the ballast to port
            ballast_speed -= (errorSum * 105.0) # the max errorSum value can be is ~0.33, so this means the max
                                                # speed can be approximately at 60
        else:
            ballast_speed += (errorSum * 105.0) # the max errorSum value can be is ~0.33, so this means the max
                                                # speed can be approximately at 130

        ballast_json = {"channel": "12", "angle": ballast_speed} # create a json file to send to the motor
        self.pwm_control_publisher_.publish(self.make_json_string(ballast_json)) # publish the json
    # ...otherwise, if we want the ballast to stay still
    ballast_json = {"channel": "12", "angle": 0} # despite being outside the range of 60-130, sending 0 stops the ballast motor for some reason
    self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))

#TO FIX: add ADC based safeguard so the ballast doesn't try to go off the rail/past the limits!
def ballast_alg_active(): # unlike the passive alg, this is designed to aim for the +/-20 degree angle at all times
    # NOTE: max port ADC value for ballast is 0.16; starboard is 0.79; midship is 0.48
    portRoll # True or False depending on whether we want to lean to the left/port (true) or right/starboard (false)
    if len(self.lastWinds) == 0:
        return # failsafe if we have received no data on wind to prevent crash
    else:
        portRoll = checkDesiredRoll()

    rollError
    if (portRoll): # if we are leaning port
        rollError = 20 + self.airmar_data["roll"] # -20 is our desired goal
    else: # if we are leaning starboard
        rollError = -20 + self.airmar_data["roll"] # 20 is our desired goal
    
    # make more efficient with rolling overwrite # used in integral error and derivative error
    # prevErrors[4] = prevErrors[3]
    # prevErrors[3] = prevErrors[2]
    # prevErrors[2] = prevErrors[1]
    # prevErrors[1] = prevErrors[0]
    # prevErrors[0] = adcError

    if (rollError > 1 or rollError < -1): # if the ballast needs to move...
        #integralAngleError = sum(prevErrors)
        #derivativeAngleError = (adcError - prevErrors[1])*0.01 # Change 0.01 to avg time between changes [or just delete derivative lol]

        errorSum = rollError * activeKp #+ integralAngleError * ki #+ derivativeAngleError * kd

        if (-3 < rollError < 3): # if the ballast is close to it's goal and needs to slow down (-2/2 was chosen arbitrarily)
            errorSum /= (1.0 + rollError/2.0) # this will linearly decrease the speed at which the ballast
                                              # is moving based on how close the ballast is to it's goal
        
        # translate the PID error into our output range; the motor accepts 60-130, with 60
        # being full tilt port and 130 being full tilt starboard; 95 is the median value
        ballast_speed = 95
        ballast_speed += (errorSum * 1.75) # the max errorSum value can be is ~20, so this means the max
                                          # speed can be approximately at 60 (or 130)

        ballast_json = {"channel": "12", "angle": ballast_speed} # create a json file to send to the motor
        self.pwm_control_publisher_.publish(self.make_json_string(ballast_json)) # publish the json
    # ...otherwise, if we want the ballast to stay still:
    ballast_json = {"channel": "12", "angle": 0} # despite being outside the range of 60-130, sending 0 stops the ballast motor for some reason
    self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))
