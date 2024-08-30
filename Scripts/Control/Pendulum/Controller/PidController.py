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
import json

from Control.Abstract.Controller import Controller
from Lib.Utils.PidServo import PidServo
from Lib.Utils.Math import util_math
from Control.Pendulum.Plant.Pendulum import Pendulum
from Lib.Utils.DataLogger import DataLogger

from DebugDataLogger import DebugDataLogger

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
        def __init__(self, Kp: float, Ki: float, Kd: float, plantParam: Pendulum.Param) -> None:
            """
            constructor

            Args:
                Kp (float): proportional gain
                Ki (float): integral gain
                Kd (float): derivative gain
                plantParam (Pendulum.Param): parameters of the plant
                
            """
            self.pidParam = PidServo.Param(Kp, Ki, Kd)
            self.plantParam = plantParam

        def __str__(self) -> str:
            return json.dumps({
                "PidController": {
                    "pidParam": json.loads(str(self.pidParam))
                }})

    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): controller parameters
        """
        super().__init__(controllerParam)
        self.pid_ = PidServo(controllerParam.pidParam)
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

        if param.pidParam.Kp != 0.0:
            self.pid_.ComputeAntiWindup(self.controlInput, prevSatInput[0], 1.0 / param.pidParam.Kp, dt)

        angleControlInput = self.pid_.ComputeControlInput(refState[0], theta, dt)

        self.controlInput = angleControlInput

        satControlInput = util_math.InvertibleSat(angleControlInput, -param.plantParam.tauMax, param.plantParam.tauMax, alpha=param.plantParam.alpha)

        return np.array([satControlInput])
    
    def PushStateToLoggerImpl(self, refState: np.array, logger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            refState (np.array): reference state
            logger (DataLogger): data logger
        """
        logger.PushData(refState[0], "refTheta")

# ----------------------------------------------------------------------------
# * @file PidController.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
