#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file polePlacementModel.py
# * @brief Reference model of PID pole placement method
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
from Control.Seminar.Plant.MassSpringDamper import MassSpringDamper

class PolePlacementModel(Plant):
    """
    Plant class

    Formula:
        d^3x/dt^3 = (p0 + p1 + p2) dx^2/dt^2 + (p0*p1 + p1*p2 + p2*p0) dx/dt + p0*p1*p2 x + u
        y = (Kd/m) d^2x/dt^2 + (Kp/m) dx/dt + (Ki/m) x

        kp = -k + m (p0*p1 + p1*p2 + p2*p0)
        ki = -m p0*p1*p2
        kd = -c - m (p0 + p1 + p2)

    Constructor:
        PolePlacementModel(plantParam, solverType, initialState)

    Methods:
        See Plant class
    """

    class Param(Plant.Param):
        """
        Parameters of plant
        """
        def __init__(self, MassSpringDamperParam: MassSpringDamper.Param, 
                     poles: np.ndarray) -> None:
            """
            constructor

            Args:
                MassSpringDamperParam (MassSpringDamper.Param): mass spring damper model parameters
                poles (np.ndarray): poles of the system

            """
            self.MassSpringDamperParam = MassSpringDamperParam
            self.poles = poles

        def __str__(self) -> str:
            return json.dumps({
                "PolePlacementModel": {
                    "MassSpringDamperModel": str(self.MassSpringDamperParam),
                    "Poles": self.poles.tolist()
                }})

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): ODE solver type
            initialState (np.ndarray, optional): initial state
        """
        stateOrder = 3
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
        p = param.poles

        diffState = np.zeros_like(curState)
        diffState[0] = curState[1]
        diffState[1] = curState[2]
        diffState[2] = p[0] * p[1] * p[2] * curState[0]\
                       - (p[0] * p[1] + p[1] * p[2] + p[2] * p[0]) * curState[1]\
                       + (p[0] + p[1] + p[2]) * curState[2]\
                       + curInput[0]

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
        pp = param.MassSpringDamperParam
        p = param.poles
        kp = -pp.springCoef + pp.mass * (p[0] * p[1] + p[1] * p[2] + p[2] * p[0])
        ki = -pp.mass * p[0] * p[1] * p[2]
        kd = -pp.viscousCoef - pp.mass * (p[0] + p[1] + p[2])

        output = kd / pp.mass * curState[2] + kp / pp.mass * curState[1] + ki / pp.mass * curState[0]

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
        graphPlotter.PushPlotYData(curInput[0], "Input", "Force")


# ----------------------------------------------------------------------------
# * @file polePlacementModel.py
# * History
# * -------
# * - 2024/10/06 New created.(By hoshina)
