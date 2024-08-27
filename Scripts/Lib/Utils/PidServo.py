#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file PidServo.py
# * @brief PID controller class
# * @author hoshina
# * @date 2024/07/02
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
import json

class PidServo:
    """
    PID controller class

    Constructor:
        PidServo(pidParam)

    Methods:
        ComputeControlInput: Compute control input
            controlInput = ComputeControlInput(refState, curState, dt)
        ComputeAntiWindup: Compute anti-windup compensation
            ComputeAntiWindup(controlInput, satControlInput, antiWindupGain, dt)
    """

    class Param():
        """
        Super class of the PID controller parameters
        """
        def __init__(self, Kp: float, Ki: float, Kd: float) -> None:
            """
            constructor

            Args:
                Kp (float): proportional gain
                Ki (float): integral gain
                Kd (float): derivative gain
            """
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd

        def __str__(self) -> str:
            return json.dumps({
                "PidServo": {
                    "Kp": self.Kp,
                    "Ki": self.Ki,
                    "Kd": self.Kd
                }})

    def __init__(self, pidParam: Param) -> None:
        """
        constructor

        Args:
            pidParam (Param): PID controller parameters
        """
        self.pidParam_ = pidParam
        self.prevError_ = 0
        self.intError_ = 0

    def ComputeControlInput(self, refState: float, curState: float, dt: float) -> float:
        """
        Compute control input

        Args:
            refState (float): reference state
            curState (float): current state
            dt (float): time step

        Returns:
            np.array: control input
        """
        error = refState - curState
        self.intError_ += error * dt
        derivative = (error - self.prevError_) / dt

        controlInput = self.pidParam_.Kp * error + self.pidParam_.Ki * self.intError_ + self.pidParam_.Kd * derivative

        self.prevError_ = error

        return controlInput
    
    def ComputeAntiWindup(self, controlInput: float, satControlInput: float, antiWindupGain: float, dt: float) -> None:
        """
        Compute anti-windup compensation.

        Args:
            controlInput (float): control input
            satControlInput (float): saturated control input
            antiWindupGain (float): anti-windup gain
            dt (float): time step
        """
        self.intError_ += (satControlInput - controlInput) * dt * antiWindupGain

# ----------------------------------------------------------------------------
# * @file PidServo.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
