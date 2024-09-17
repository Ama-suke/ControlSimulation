#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file DcMotor.py
# * @brief DC motor model
# * @author hoshina
# * @date 2024/07/01
# * @details
# *
# ----------------------------------------------------------------------------

import numpy as np
from Control.Abstract.Plant import Plant
from Lib.Compensator.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger

class DcMotor(Plant):
    """
    DC motor model

    Constructor:
        DcMotor(plantParam, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None)

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
        Parameters of DC motor
        """
        def __init__(self, J, D, R, L, Phi, uMax, uMin, vNoise) -> None:
            """
            constructor

            Args:
                J (float): inertia
                D (float): damping coefficient
                R (float): resistance
                L (float): inductance
                Phi (float): magnetic flux
                uMax (float): maximum input
                uMin (float): minimum input
                vNoise (float): variance of noise
            """
            self.J = J
            self.D = D
            self.R = R
            self.L = L
            self.Phi = Phi
            self.uMax = uMax
            self.uMin = uMin
            self.vNoise = vNoise

    def __init__(self, plantParam: Param,
                 solverType = StateSpace.SolverType.RUNGE_KUTTA,
                 initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): solver type. Defaults to StateSpace.SolverType.RUNGE_KUTTA.
            initialState (np.ndarray, optional): initial state. Defaults to zero.
        """
        super().__init__(2, plantParam, solverType, initialState)

    def StateEquation(self, curState: np.ndarray, curInput: np.ndarray, param) -> np.ndarray:
        """
        State equation of DC motor
        
        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input

        Returns:
            np.ndarray: differential state
        """
        J = param.J
        D = param.D
        R = param.R
        L = param.L
        Phi = param.Phi

        omega = curState[0]
        i = curState[1]
        voltage = self.GetSaturatedInput(curInput)[0]
        
        di = (1 / L) * (voltage - R * i - Phi * omega)
        domega = (1 / J) * (Phi * i - D * omega)

        return np.array([domega, di])

    def OutputEquation(self, curState: np.ndarray, curInput: np.ndarray, param) -> np.ndarray:
        """
        Output equation of DC motor

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input

        Returns:
            np.ndarray: output
        """
        noise = np.random.normal(np.zeros(2), param.vNoise)[0]
        return self.GetState() + noise
    
    def GetSaturatedInputImpl(self, curInput: np.ndarray) -> np.ndarray:
        """
        Get the saturated input

        Args:
            curInput (np.ndarray): current input

        Returns:
            np.ndarray: saturated input
        """
        u = curInput[0]
        if u > 1.0:
            u = 1.0
        elif u < -1.0:
            u = -1.0
        return np.array([u])
    
    def GetSaturatedInputImpl(self, u: np.ndarray, param: np.ndarray) -> np.ndarray:
        """
        Get the saturated input

        Args:
            u (np.ndarray): input
            param (np.ndarray): parameters

        Returns:
            np.ndarray: saturated input
        """
        uMax = param.uMax
        uMin = param.uMin

        return np.array([np.clip(u[0], uMin, uMax)])
    
# ----------------------------------------------------------------------------
# * @file DcMotor.py
# * History
# * -------
# * - 2024/08/01 New created.(By hoshina)