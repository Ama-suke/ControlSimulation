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
        GetStates: Get the states (indexes: 0 error, 1 diffError, 2 intError)
            states = GetStates()
    """

    class Param():
        """
        Super class of the PID controller parameters
        """
        def __init__(self, Kp: float, Ki: float, Kd: float, tau: float = 0) -> None:
            """
            constructor

            Args:
                Kp (float): proportional gain
                Ki (float): integral gain
                Kd (float): derivative gain
                tau (float): time constant of the low-pass filter. If 0, no filter is used.
            """
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.tau = tau

        def __str__(self) -> str:
            return json.dumps({
                "PidServo": {
                    "Kp": self.Kp,
                    "Ki": self.Ki,
                    "Kd": self.Kd,
                    "tau": self.tau
                }})

    def __init__(self, pidParam: Param) -> None:
        """
        constructor

        Args:
            pidParam (Param): PID controller parameters
        """
        self.param_ = pidParam
        self.error_ = 0
        self.prevError_ = 0
        self.diffError_ = 0
        self.intError_ = 0

    def ComputeControlInput(self, refState: float, curState: float, dt: float) -> float:
        """
        Compute control input

        Args:
            refState (float): reference state
            curState (float): current state
            dt (float): time step

        Returns:
            np.ndarray: control input
        """
        lpfCoef = self.param_.tau / (dt  + self.param_.tau)

        self.error = refState - curState
        self.intError_ += self.error * dt
        self.diffError_ = (self.error - self.prevError_) / dt
        self.prevError_ = lpfCoef * self.prevError_ + (1 - lpfCoef) * self.error

        controlInput = self.param_.Kp * self.error + self.param_.Ki * self.intError_ + self.param_.Kd * self.diffError_

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

    def GetStates(self) -> np.ndarray:
        """
        Get the states

        Returns:
            np.ndarray: the states
        """
        return np.array([self.error, self.diffError_, self.intError_])

# ----------------------------------------------------------------------------
# * @file PidServo.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
# * - 2024/09/18 Add differential filter.(By hoshina)
