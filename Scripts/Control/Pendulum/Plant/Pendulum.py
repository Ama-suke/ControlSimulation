#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Pendulum.py
# * @brief 振り子の数学モデル
# * @author hoshina
# * @date 2024/08/22
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np

from Control.Abstract.Plant import Plant
from Lib.Utils.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger

class Pendulum(Plant):
    """
    振り子の数学モデル

    Constructor:
        Pendulum(plantParam, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None)
    """

    class Param(Plant.Param):
        """
        Parameters of inverted wheel pendulum
        """
        def __init__(self, m, J, D, l, g, tauMin, tauMax) -> None:
            """
            constructor

            Args:
                m (float): mass
                J (float): moment of inertia
                D (float): damping coefficient
                l (float): length
                g (float): gravitational acceleration
                tauMin (float): minimum input. Set "None" if no limit
                tauMax (float): maximum input. Set "None" if no limit
            """
            self.m = m
            self.J = J
            self.D = D
            self.l = l
            self.g = g
            self.tauMin = tauMin
            self.tauMax = tauMax

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): solver type. Defaults to StateSpace.SolverType.RUNGE_KUTTA.
            initialState (np.array, optional): initial state. Defaults to zero.
        """
        stateOrder = 2
        super().__init__(stateOrder, plantParam, solverType, initialState)

    # private ------------------------------------------------------
    def StateEquation(self, curState, curInput, param):
        """
        State equation of pendulum

        Args:
            curState (np.array): current state
            curInput (np.array): current input
            param (Param): parameters

        Returns:
            np.array: derivative of the state
        """
        m = param.m
        J = param.J
        D = param.D
        l = param.l
        g = param.g
        tau = self.GetSaturatedInput(curInput)[0]

        theta = curState[0]
        omega = curState[1]

        dTheta = omega
        dOmega = (tau + m * g * l * np.sin(theta) - D * omega) / J

        return np.array([dTheta, dOmega])
    
    def OutputEquation(self, curState, curInput, param):
        """
        Output equation of inverted wheel pendulum

        Args:
            curState (np.array): current state
            curInput (np.array): current input
            param (Param): parameters

        Returns:
            np.array: output
        """
        return np.array([curState[0]])
    
    def GetSaturatedInputImpl(self, u: np.array, param: np.array) -> np.array:
        """
        Get the saturated input.
        
        Args:
            u (np.array): input
            param (np.array): parameters

        Returns:
            np.array: saturated input
        """
        tauMin = param.tauMin
        tauMax = param.tauMax

        return np.array([np.clip(u[0], tauMin, tauMax)])
    
    def PushStateToLoggerImpl(self, curInput: np.array, logger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            curInput (np.array): current input
            logger (DataLogger): logger
        """
        logger.PushData(self.stateVariable_[0], "theta")
        logger.PushData(self.stateVariable_[1], "omega")
        logger.PushData(curInput[0], "tau")

# ----------------------------------------------------------------------------
# * @file Pendulum.py
# * History
# * -------
# * - 2024/08/22 New created.(By hoshina)
