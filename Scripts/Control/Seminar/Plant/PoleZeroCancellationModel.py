#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file PoleZeroCancellationModel.py
# * @brief Pole-zero cancellation model
# * @author hoshina
# * @date 2024/10/06
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Plant import Plant

import numpy as np
import json

from Lib.Compensator.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter
from DebugDataLogger import DebugDataLogger

class PoleZeroCancellationModel(Plant):
    """
    Plant class

    Formula:
        dy/dt = omega (-y + u)

    Constructor:
        PoleZeroCancellationModel(plantParam, solverType, initialState)

    Methods:
        See Plant class
    """

    class Param(Plant.Param):
        """
        Parameters of plant
        """
        def __init__(self, omega) -> None:
            """
            constructor

            Args:
                omega (float): natural frequency
            """
            self.omega = omega

        def __str__(self) -> str:
            return json.dumps({
                "PoleZeroCancellationModel": {
                    "omega": self.omega
                }})

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): ODE solver type
            initialState (np.ndarray, optional): initial state
        """
        stateOrder = 1
        super().__init__(stateOrder, plantParam, solverType, initialState)

    # private ------------------------------------------------------
    def StateEquation(self, curState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        """
        State equation of the plant

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): plant parameters

        Returns:
            np.ndarray: derivative of the state
        """
        diffState = np.zeros_like(curState)
        diffState[0] = param.omega * (-curState[0] + curInput[0])

        return diffState

    def OutputEquation(self, curState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        """
        Output equation of the plant

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): plant parameters

        Returns:
            np.ndarray: output of the plant
        """
        output = curState[0]
        return np.array([output])

    def GetSaturatedInputImpl(self, controlInput: np.ndarray, param: Param) -> np.ndarray:
        """
        Get saturated control input

        Args:
            controlInput (np.ndarray): control input
            param (Param): plant parameters

        Returns:
            np.ndarray: saturated input
        """
        return controlInput

    def PushStateToLoggerImpl(self, curInput: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            curInput (np.ndarray): current input
            dataLogger (DataLogger): data logger
        """
        dataLogger.PushData(curInput[0], "InputForce")

    def PushStateForPlotImpl(self, curInput: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the state to the plotter

        Args:
            curInput (np.ndarray): current input
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PushPlotYData(curInput[0], "InputForce", "Force")


# ----------------------------------------------------------------------------
# * @file PoleZeroCancellationModel.py
# * History
# * -------
# * - 2024/10/06 New created.(By hoshina)
