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
from Lib.Compensator.PidServo import PidServo
from Lib.Utils.Math import util_math
from Control.SimplePendulum.Plant.Pendulum import Pendulum
from Lib.Utils.DataLogger import DataLogger

from DebugDataLogger import DebugDataLogger
from Lib.Utils.GraphPlotter import GraphPlotter

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
        def __init__(self, pidParam: PidServo.Param, plantParam: Pendulum.Param) -> None:
            """
            constructor

            Args:
                pidParam (PidServo.Param): parameters of the PID controller
                plantParam (Pendulum.Param): parameters of the plant
                
            """
            self.pidParam = pidParam
            self.plantParam = plantParam

        def __str__(self) -> str:
            return json.dumps({
                "PidController": {
                    "pidParam": json.loads(str(self.pidParam)),
                    "plantParam": json.loads(str(self.plantParam))
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
    def ComputeControlInputImpl(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute control input

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            prevSatInput (np.ndarray): previous saturated input
            dt (float): time step
            param (Param):  controller parameters

        Returns:
            np.ndarray: control input
        """
        theta = curState[0]

        if param.pidParam.Kp != 0.0:
            self.pid_.ComputeAntiWindup(self.controlInput, prevSatInput[0], 1.0 / param.pidParam.Kp, dt)

        angleControlInput = self.pid_.ComputeControlInput(refState[0], theta, dt)

        self.controlInput = angleControlInput

        satControlInput = util_math.InvertibleSat(angleControlInput, -param.plantParam.tauMax, param.plantParam.tauMax, alpha=param.plantParam.alpha)

        return np.array([satControlInput])
    
    def PushStateToLoggerImpl(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            refState (np.ndarray): reference state
            dataLogger (DataLogger): data logger
        """
        dataLogger.PushData(refState[0], "Ref")
        dataLogger.PushData(refState[1] - curState[0], "Error")

    def PushStateForPlotImpl(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the state to the graph plotter

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PlotData(refState[0], "Ref")
        graphPlotter.PlotData(refState[1] - curState[0], "Error")

        return

# ----------------------------------------------------------------------------
# * @file PidController.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
