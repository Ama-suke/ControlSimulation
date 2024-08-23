#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file PidController.py
# * @brief PID controller for inverted wheel pendulum
# * @author hoshina
# * @date 2024/07/02
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from numpy.core.multiarray import array as array

from Control.Abstract.Controller import Controller
from Lib.Utils.PidServo import PidServo
from Lib.Utils.DataLogger import DataLogger

class PidController(Controller):
    """
    PID controller class

    Constructor:
        PidController(controllerParam)

    Methods:
        ComputeControlInput: Compute control input
            controlInput = ComputeControlInput(refState, curState, prevSatInput, dt)
    """

    class Param(Controller.Param):
        """
        Parameters of PID controller
        """
        def __init__(self, KpAngle: float, KiAngle: float, KdAngle: float, KpPos: float, KiPos: float, KdPos: float) -> None:
            """
            constructor

            Args:
                KpAngle (float): proportional gain for angle
                KiAngle (float): integral gain for angle
                KdAngle (float): derivative gain for angle
                KpPos (float): proportional gain for displacement
                KiPos (float): integral gain for displacement
                KdPos (float): derivative gain for displacement
                
            """
            self.anglePidParam = PidServo.Param(
                KpAngle, KiAngle, KdAngle)
            self.posPidParam = PidServo.Param(
                KpPos, KiPos, KdPos)

    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): controller parameters
        """
        super().__init__(controllerParam)
        self.anglePid_ = PidServo(controllerParam.anglePidParam)
        self.posPid_ = PidServo(controllerParam.posPidParam)
        self.controlInput = 0.0

    # private ------------------------------------------------------
    def ComputeControlInputImpl(self, refState: np.array, curState: np.array, prevSatInput: np.array, dt: float, param: Param) -> np.array:
        """
        Compute control input

        Args:
            refState (np.array): reference state
            curState (np.array): current state
            prevSatInput (np.array): previous saturated input
            dt (float): time step
            param (Param):  controller parameters

        Returns:
            np.array: control input
        """
        theta = curState[0]
        phi = curState[1]

        if param.anglePidParam.Kp != 0.0:
            self.anglePid_.ComputeAntiWindup(self.controlInput, prevSatInput[0], 1.0 / param.anglePidParam.Kp, dt)
        if param.posPidParam.Kp != 0.0:
            self.posPid_.ComputeAntiWindup(self.controlInput, prevSatInput[0], 1.0 / param.posPidParam.Kp, dt)

        angleControlInput = self.anglePid_.ComputeControlInput(0, -theta, dt)
        posControlInput = self.posPid_.ComputeControlInput(refState[0], phi, dt)

        self.controlInput = angleControlInput + posControlInput

        return np.array([self.controlInput])
    
    def PushStateToLoggerImpl(self, refState: np.array, dataLogger: DataLogger) -> None:
        dataLogger.PushData(refState[0], "refX")

# ----------------------------------------------------------------------------
# * @file PidController.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
