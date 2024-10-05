#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file PidPositonController.py
# * @brief PID位置制御器クラス
# * @author hoshina
# * @date 2024/10/05
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Controller import Controller

import numpy as np
import json

from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter
from DebugDataLogger import DebugDataLogger
from Lib.Compensator.PidServo import PidServo

class PidPositionController(Controller):
    """
    PID controller class

    Constructor:
        PidPositionController(controllerParam)

    Methods:
        See Controller class
    """

    class Param(Controller.Param):
        """
        Parameters of PID controller
        """
        def __init__(self, pidParam: PidServo.Param) -> None:
            """
            constructor

            Args:
                pidParam (PidServo.Param): PID parameters
            """
            self.pidParam = pidParam

        def __str__(self) -> str:
            return json.dumps({
                "PidPositionController": {
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

    # private ------------------------------------------------------
    def ComputeControlInputImpl(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute control input

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current sensing state
            prevSatInput (np.ndarray): previous saturated input
            dt (float): time step
            param (Param):  controller parameters

        Returns:
            np.ndarray: control input
        """
        controlInput = self.pid_.ComputeControlInput(refState[0], curState[0], dt)

        return np.array([controlInput])

    def PushStateToLoggerImpl(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current sensing state
            dataLogger (DataLogger): data logger
        """
        dataLogger.PushData(refState[0], "Ref")
        dataLogger.PushData(refState[0] - curState[0], "Error")

    def PushStateForPlotImpl(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        graphPlotter.PushPlotYData(refState[0], "Ref", "Position")    

# ----------------------------------------------------------------------------
# * @file PidPositonController.py
# * History
# * -------
# * - 2024/10/05 New created.(By hoshina)
