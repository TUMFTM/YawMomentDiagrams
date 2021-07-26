import numpy as np

"""
Python version: 3.8
Created by: Frederik Werner
Created on: 01.02.2021
"""

# ------------------------------------------------------------------------------
class PID:
    dt = 0.0
    max = 0.0
    min = 0.0
    kp = 0.0
    kd = 0.0
    ki = 0.0
    err = 0.0
    int = 0.0

    def __init__(self, params):
        self.dt = params[0]
        self.max = params[1]
        self.min = params[2]
        self.kp = params[3]
        self.kd = params[4]
        self.ki = params[5]

    def run(self, set, act):
        # calculate control error
        error = set - act

        #calculate p-part
        p = self.kp * error

        # integrate control error
        self.int += error * self.dt

        #calculate i-part
        i = self.ki * self.int

        #calculate d-part
        d = self.kd * (error - self.err) / self.dt

        # calculate controller output
        output = p + i + d

        # check saturation limits
        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min

        # log error for next step
        self.err = error
        return output


def torque_arr(DriveTorque):
    arr = np.zeros(4)
    arr[2] = 0.5*DriveTorque
    arr[3] = 0.5*DriveTorque
    return arr

def brake_arr(BrakePressure, veh_config):
    arr = np.zeros(4)
    p_front = veh_config["brake_balance"] * BrakePressure
    p_rear = (1-veh_config["brake_balance"]) * BrakePressure
    arr[0] = p_front
    arr[1] = p_front
    arr[2] = p_rear
    arr[3] = p_rear
    return arr
