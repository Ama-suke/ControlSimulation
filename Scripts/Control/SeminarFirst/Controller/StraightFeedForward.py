#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file StraightFeedForward.py
# * @brief control input is reference input itself
# * @author hoshina
# * @date 2024/10/06
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Controller import Controller

import numpy as np
import json

from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter
from DebugDataLogger import DebugDataLogger

class StraightFeedForward(Controller):
    """
    PID controller class

    Constructor:
        StraightFeedForward(controllerParam)

    Methods:
        See Controller class
    """

    class Param(Controller.Param):
        """
        Parameters of PID controller
        """
        def __init__(self) -> None:
            """
            constructor

            Args:

            """

        def __str__(self) -> str:
            return ""

    def __init__(self) -> None:
        """
        constructor

        Args:
        """
        super().__init__(None)

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
        controlInput = refState[0]
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
        dataLogger.PushData(curState[0], "Output")
        dataLogger.PushData(refState[0] - curState[0], "Error")

    def PushStateForPlotImpl(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the state to the plotter

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current sensing state
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PushPlotYData(refState[0], "Ref", "Position")
        graphPlotter.PushPlotYData(curState[0], "Output", "Position")
        graphPlotter.PushPlotYData(refState[0] - curState[0], "Error", "Error")


# ----------------------------------------------------------------------------
# * @file StraightFeedForward.py
# * History
# * -------
# * - 2024/10/06 New created.(By hoshina)
