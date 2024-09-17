#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file InvertedWheelPendulum.py
# * @brief Mathematical model of inverted wheel pendulum 
# * @author hoshina
# * @date 2024/07/01
# * @sa readme.md
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np

from Control.Abstract.Plant import Plant
from Lib.Compensator.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger

class InvertedWheelPendulum(Plant):
    """
    Plant class of inverted wheel pendulum

    Constructor:
        InvertedWheelPendulum(plantParam, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None)

    Methods:
        UpdateState: Update the state of the plant
            UpdateState(curInput, dt)
        GetState: Get the current state
            GetState()
        GetOutput: Get the output of the plant
            GetOutput(curInput)
        GetSaturatedInput: Get the saturated input
            GetSaturatedInput(curInput)
    """

    class Param(Plant.Param):
        """
        Parameters of inverted wheel pendulum
        """
        def __init__(self, m_w, J_w, D_w, m_p, J_p, D_p, r, l_g, g, tauMin, tauMax) -> None:
            """
            constructor

            Args:
                m_w (float): mass of the wheel
                J_w (float): moment of inertia of the wheel
                D_w (float): damping coefficient of the wheel
                m_p (float): mass of the pendulum
                J_p (float): moment of inertia of the pendulum
                D_p (float): damping coefficient of the pendulum
                r (float): radius of the wheel
                l_g (float): distance from the wheel to the center of gravity of the pendulum
                g (float): gravitational acceleration
                tauMin (float): minimum input. Set "None" if no limit
                tauMax (float): maximum input. Set "None" if no limit
            """
            self.m_w = m_w
            self.J_w = J_w
            self.D_w = D_w
            self.m_p = m_p
            self.J_p = J_p
            self.D_p = D_p
            self.r = r
            self.l_g = l_g
            self.g = g
            self.tauMin = tauMin
            self.tauMax = tauMax

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): solver type. Defaults to StateSpace.SolverType.RUNGE_KUTTA.
            initialState (np.ndarray, optional): initial state. Defaults to zero.
        """
        stateOrder = 4
        super().__init__(stateOrder, plantParam, solverType, initialState)

    # private ------------------------------------------------------
    def StateEquation(self, curState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        """
        State equation of inverted wheel pendulum

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): parameters

        Returns:
            np.ndarray: derivative of the state
        """
        m_w = param.m_w
        J_w = param.J_w
        D_w = param.D_w
        m_p = param.m_p
        J_p = param.J_p
        D_p = param.D_p
        r = param.r
        l_g = param.l_g
        g = param.g

        theta = curState[0]     # angle of the pendulum
        omega = curState[1]     # angular velocity of the pendulum
        phi = curState[2]       # angle of the wheel
        omega_w = curState[3]   # angular velocity of the wheel
        tau = self.GetSaturatedInput(curInput)[0]
        dist = curInput[1]

        inertiaMatrix = np.array([[J_p + J_w + (m_w + m_p) * r ** 2 + m_p * l_g ** 2 + 2 * m_p * r * l_g * np.cos(theta),
                                   J_w + (m_w + m_p) * r ** 2 + m_p * r * l_g * np.cos(theta)],
                                  [J_w + (m_w + m_p) * r ** 2 + m_p * r * l_g * np.cos(theta),
                                   J_w + (m_w + m_p) * r ** 2]
                                   ])
        moment = np.array([-D_p * omega + m_p * l_g * (g + r * omega ** 2) * np.sin(theta),
                           -D_w * omega_w + m_p * r * l_g * omega ** 2 * np.sin(theta) + tau - dist])
        
        dTheta = omega
        dPhi = omega_w
        ddTheta, ddPhi = np.linalg.solve(inertiaMatrix, moment)

        return np.array([dTheta, ddTheta, dPhi, ddPhi])
    
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
        m_w = param.m_w
        J_w = param.J_w
        m_p = param.m_p
        J_p = param.J_p
        r = param.r
        l_g = param.l_g
        g = param.g

        theta = curState[0]
        omega = curState[1]
        phi = curState[2]
        omega_w = curState[3]
        tau = self.GetSaturatedInput(curInput)[0]

        displacement = r * (phi + theta)

        return np.array([theta, displacement])
    
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
            logger (DataLogger): logger
        """
        y = self.GetOutput(curInput)
        logger.PushData(self.GetState()[0], "theta")
        logger.PushData(self.GetState()[1], "omega")
        logger.PushData(self.GetState()[2], "phi")
        logger.PushData(self.GetState()[3], "omega_w")
        logger.PushData(y[1], "x")
        logger.PushData(curInput[0], "tau")
        logger.PushData(curInput[1], "dist")
    
# ----------------------------------------------------------------------------
# * @file InvertedWheelPendulum.py
# * History
# * -------
# * - 2024/07/01 New created.(By hoshina)
