#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * @brief Operator controller for inverted wheel pendulum
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np

from Control.Abstract.Controller import Controller
from Lib.Utils.PidServo import PidServo

class OperatorController(Controller):
    """
    Operator controller class

    Constructor:
        OperatorController(controllerParam)

    Methods:
        ComputeControlInput: Compute control input
            controlInput = ComputeControlInput(refState, curState, prevSatInput, dt)
    """

    class Param(Controller.Param):
        """
        Parameters of operator controller
        """
        def __init__(self, Kp: float, Ki: float, Kd: float) -> None:
            """
            constructor

            Args:
                Kp (float): proportional gain
                Ki (float): integral gain
                Kd (float): derivative gain
                
            """
            self.pidParam = PidServo.Param(Kp, Ki, Kd)

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
            param (Param): controller parameters

        Returns:
            np.array: control input
        """
        return self.pid_.ComputeControlInput(refState, curState, prevSatInput, dt)

# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
