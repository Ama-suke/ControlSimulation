#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file MassSpringDamper.py
# * @brief ばね-質点-ダンパ系の数学モデル
# * @author hoshina
# * @date 2024/10/05
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

class MassSpringDamper(Plant):
    """
    Plant class

    Formula:
        d^2y/dt^2 = -D/M dy/dt - K/M y + u

        K = spring coef
        D = viscous coef
        M = mass
        y = system position
        u = input force

    Constructor:
        MassSpringDamper(plantParam, solverType = StateSpace, initialState)

    Methods:
        See Plant class
    """

    class Param(Plant.Param):
        """
        Parameters of plant
        """
        def __init__(self, mass, springCoef, viscousCoef) -> None:
            """
            constructor

            Args:
                mass (float): mass
                springCoef (float): spring coefficient
                viscousCoef (float): viscous coefficient
            """
            self.mass = mass
            self.springCoef = springCoef
            self.viscousCoef = viscousCoef

        def __str__(self) -> str:
            return json.dumps({
                "MassSpringDamper": {
                    "mass": self.mass,
                    "springCoef": self.springCoef,
                    "viscousCoef": self.viscousCoef
                }})

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): ODE solver type
            initialState (np.ndarray, optional): initial state
        """
        stateOrder = 2
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

        diffState[0] = curState[1]
        diffState[1] = (curInput[0] - param.springCoef * curState[0] - param.viscousCoef * curState[1]) / param.mass

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
    
    def GetSaturatedInputImpl(self, u: np.ndarray, param: np.ndarray) -> np.ndarray:
        return u

    def PushStateToLoggerImpl(self, curInput: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            curInput (np.ndarray): current input
            dataLogger (DataLogger): data logger
        """
        dataLogger.PushData(self.stateVariable_[0], "Position")
        dataLogger.PushData(self.stateVariable_[1], "Velocity")
        dataLogger.PushData(curInput[0], "InputForce")

    def PushStateForPlotImpl(self, curInput: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the data to the graph plotter

        Args:
            curInput (np.ndarray): current input
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PushPlotYData(self.stateVariable_[0], "Position", "Position")
        graphPlotter.PushPlotYData(self.stateVariable_[1], "Velocity", "Velocity")
        graphPlotter.PushPlotYData(curInput[0], "InputForce", "Force")

# ----------------------------------------------------------------------------
# * @file MassSpringDamper.py
# * History
# * -------
# * - 2024/10/05 New created.(By hoshina)
