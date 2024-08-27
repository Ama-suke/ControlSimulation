#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file EquivalentSaturateModel.py
# * @brief Equivalent saturation model for inverted wheel pendulum
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
import json

from Control.Abstract.Plant import Plant
from Lib.Utils.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.Math import util_math

from DebugDataLogger import DebugDataLogger

class EquivalentSaturateModel(Plant):
    """
    Equivalent saturation model for inverted wheel pendulum

    Constructor:
        EquivalentSaturateModel(plantParam, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None)
    """

    class Param(Plant.Param):
        """
        Parameters of inverted wheel pendulum
        """
        def __init__(self, m, J, D, l, g, tauMin, tauMax, Kp, Kd, wMin, wMax, deltaDDisabled) -> None:
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
                Kp (float): proportional gain
                Kd (float): derivative gain
                wMin (float): minimum quasi-state
                wMax (float): maximum quasi-state
                deltaDDisabled (bool): True if the uncertainty ΔD is disabled
            """
            self.m = m
            self.J = J
            self.D = D
            self.l = l
            self.g = g
            self.tauMin = tauMin
            self.tauMax = tauMax
            self.Kp = Kp
            self.Kd = Kd
            self.wMin = wMin
            self.wMax = wMax
            self.deltaDDisabled = deltaDDisabled

        def __str__(self) -> str:
            return json.dumps({
                "EquivalentPendulum": {
                    "m": self.m,
                    "J": self.J,
                    "D": self.D,
                    "l": self.l,
                    "g": self.g,
                    "tauMin": self.tauMin,
                    "tauMax": self.tauMax,
                    "Kp": self.Kp,
                    "Kd": self.Kd,
                    "wMin": self.wMin,
                    "wMax": self.wMax,
                    "deltaDDisabled": self.deltaDDisabled
                }
            })

    def __init__(self, plantParam: Param, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None) -> None:
        """
        constructor

        Args:
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): solver type. Defaults to StateSpace.SolverType.RUNGE_KUTTA.
            initialState (np.array, optional): initial state. Defaults to zero.
        """
        stateOrder = 6
        initialStateArray = np.tile(initialState, int(stateOrder / 2))
        super().__init__(stateOrder, plantParam, solverType, initialStateArray)

        self.w = np.array([0.0])

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
        Kp = param.Kp
        Kd = param.Kd
        tau = curInput[0]
        dist = curInput[1]

        thetaN = curState[0]
        omegaN = curState[1]
        thetaD = curState[2]
        omegaD = curState[3]
        w = self.w[0]

        # ΔD
        deltaDState = curState[2:6]
        dDeltaDState, deltaU = self.ComputeDeltaD(deltaDState, self.w, param)
        if param.deltaDDisabled:
            deltaU = 0.0

        DebugDataLogger.PushData(deltaU, "deltaU")

        # D^-1
        w = thetaD + (tau + dist - deltaU + Kd * omegaD + m * g * l * np.sin(thetaD)) / Kp
        dThetaD = omegaD
        dOmegaD = (-(D + Kd) * omegaD - Kp * (thetaD - w)) / J

        DebugDataLogger.PushData(w, "quasi-state")
        DebugDataLogger.PushData(thetaD, "thetaD")

        # saturation
        wSat = util_math.ComputeInvertibleSat(w, param.wMin, param.wMax)
        DebugDataLogger.PushData(wSat, "quasi-stateSat")

        # N
        dThetaN = omegaN
        dOmegaN = (-(D + Kd) * omegaN - Kp * (thetaN - wSat)) / J

        # store quasi-state
        self.w[0] = wSat

        return np.concatenate((np.array([dThetaN, dOmegaN, dThetaD, dOmegaD]), dDeltaDState))
    
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

    def ComputeDeltaD(self, curState, curQuasiState, param) -> np.array:
        """
        Compute the state equation of ΔD
        formula : ΔD = σ_u^-1 D σ_w - D

        return:
            np.array: derivative of the state
            float: output of ΔD
        """

        m = param.m
        J = param.J
        D = param.D
        l = param.l
        g = param.g
        Kp = param.Kp
        Kd = param.Kd

        thetaD1 = curState[0]
        omegaD1 = curState[1]
        thetaD2 = curState[2]
        omegaD2 = curState[3]
        w = curQuasiState[0]

        # D
        u1 = Kp * (thetaD1 - w) - Kd * omegaD1 - m * g * l * np.sin(thetaD1)

        # σ_u^-1 D σ_w
        wSat = util_math.ComputeInvertibleSat(w, param.wMin, param.wMax)
        u2Sat = Kp * (thetaD2 - wSat) - Kd * omegaD2 - m * g * l * np.sin(thetaD2)
        dThetaD2 = omegaD2
        dOmegaD2 = (-(D + Kd) * omegaD2 - Kp * (thetaD2 - wSat)) / J
        u2 = util_math.ComputeInvertibleSatInv(u2Sat, param.tauMin, param.tauMax)

        DebugDataLogger.PushData(u2, "tau2DeltaD")

        u = u2 - u1

        return np.array([dThetaD2, dOmegaD2]), u

# ----------------------------------------------------------------------------
# * @file EquivalentSaturateModel.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
