#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Pendulum.py
# * @brief 振り子の数学モデル
# * @author hoshina
# * @date 2024/08/22
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Plant import Plant

import numpy as np
import json

from Lib.Compensator.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter

class Pendulum(Plant):
    """
    振り子の数学モデル

    Constructor:
        Pendulum(plantParam, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None)
    """

    class Param(Plant.Param):
        """
        Parameters of pendulum
        """
        def __init__(self, m, J, B, l, g, f0, vSigma, R, tauMin, tauMax, alpha) -> None:
            """
            constructor

            Args:
                m (float): mass
                J (float): moment of inertia
                B (float): damping coefficient
                l (float): length
                g (float): gravitational acceleration
                f0 (float): friction coefficient
                vSigma (float): stribeck velocity
                R (float): reduction ratio
                tauMin (float): minimum input. Set "None" if no limit
                tauMax (float): maximum input. Set "None" if no limit
                alpha (float): shape parameter of the saturation function
            """
            self.m = m
            self.J = J
            self.B = B
            self.l = l
            self.g = g
            self.f0 = f0
            self.vSigma = vSigma
            self.R = R
            self.tauMin = tauMin
            self.tauMax = tauMax
            self.alpha = alpha

        def __str__(self) -> str:
            return json.dumps({
                "Pendulum": {
                    "m": self.m,
                    "J": self.J,
                    "D": self.B,
                    "l": self.l,
                    "g": self.g,
                    "f0": self.f0,
                    "k": self.vSigma,
                    "R": self.R,
                    "tauMin": self.tauMin,
                    "tauMax": self.tauMax,
                    "alpha": self.alpha
                }
            })

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): solver type. Defaults to StateSpace.SolverType.RUNGE_KUTTA.
            initialState (np.ndarray, optional): initial state. Defaults to zero.
        """
        stateOrder = 2
        super().__init__(stateOrder, plantParam, solverType, initialState)

    # private ------------------------------------------------------
    def StateEquation(self, curState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        """
        State equation of pendulum

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): parameters

        Returns:
            np.ndarray: derivative of the state
        """
        m = param.m
        J = param.J
        B = param.B
        l = param.l
        g = param.g
        f0 = param.f0
        vSigma = param.vSigma
        R = param.R
        tau = self.GetSaturatedInput(curInput)[0]
        dist = curInput[1]

        theta = curState[0]
        omega = curState[1]

        torque = R * tau + dist + self.ComputeFrictionTorque(omega, tau, param)

        dTheta = omega
        dOmega = (torque + m * g * l * np.sin(theta) - B * omega) / J

        return np.array([dTheta, dOmega])
    
    def OutputEquation(self, curState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        """
        Output equation of inverted wheel pendulum

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): parameters

        Returns:
            np.ndarray: output
        """
        return np.array([curState[0]])
    
    def GetSaturatedInputImpl(self, u: np.ndarray, param: Param) -> np.ndarray:
        """
        Get the saturated input.
        
        Args:
            u (np.ndarray): input
            param (np.ndarray): parameters

        Returns:
            np.ndarray: saturated input
        """
        tauMin = param.tauMin
        tauMax = param.tauMax

        return np.array([np.clip(u[0], tauMin, tauMax)])
    
    def PushStateToLoggerImpl(self, curInput: np.ndarray, logger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            curInput (np.ndarray): current input
            logger (DataLogger): logger
        """
        logger.PushData(self.stateVariable_[0], "theta")
        logger.PushData(self.stateVariable_[1], "omega")
        logger.PushData(curInput[0], "tau")

    def PushStateForPlotImpl(self, curInput: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the state to the logger for plot

        Args:
            curInput (np.ndarray): current input
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PushPlotYData(self.stateVariable_[0], "theta", "theta")
        graphPlotter.PushPlotYData(self.stateVariable_[1], "omega", "omega")
        graphPlotter.PushPlotYData(curInput[0], "tau", "tau")

    def ComputeFrictionTorque(self, omega, tau, param: Param):
        """
        Compute friction torque

        Args:
            omega (float): angular velocity
            param (Param): parameters

        Returns:
            float: friction torque
        """
        f0 = param.f0
        vSigma = param.vSigma

        if omega == 0:
            return -np.min([f0, np.abs(tau)]) * np.sign(tau)

        return -f0 * np.exp(-np.abs(omega) / vSigma) * np.sign(omega)

# ----------------------------------------------------------------------------
# * @file Pendulum.py
# * History
# * -------
# * - 2024/08/22 New created.(By hoshina)
