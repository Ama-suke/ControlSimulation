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
from Control.SimplePendulum.Plant.Pendulum import Pendulum
from Lib.Compensator.StateSpace import StateSpace
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
        def __init__(self, baseParam: Pendulum.Param, Kp, Kd, wMin, wMax, deltaDDisabled) -> None:
            """
            constructor

            Args:
                baseParam (Pendulum.Param): base parameters
                Kp (float): proportional gain
                Kd (float): derivative gain
                wMin (float): minimum value of quasi-state
                wMax (float): maximum value of quasi-state
                deltaDDisabled (bool): True if the uncertainty ΔD is disabled
            """
            self.base = baseParam
            self.Kp = Kp
            self.Kd = Kd
            self.wMin = wMin
            self.wMax = wMax
            self.deltaDDisabled = deltaDDisabled

        def __str__(self) -> str:
            return json.dumps({
                "EquivalentPendulum": {
                    "base": json.loads(str(self.base)),
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
            initialState (np.ndarray, optional): initial state. Defaults to zero.
        """
        stateOrder = 6
        initialStateArray = np.tile(initialState, int(stateOrder / 2))
        super().__init__(stateOrder, plantParam, solverType, initialStateArray)

        self.w = np.array([0.0])

    # private ------------------------------------------------------
    def StateEquation(self, curState, curInput, param: Param):
        """
        State equation of pendulum

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): parameters

        Returns:
            np.ndarray: derivative of the state
        """
        m = param.base.m
        J = param.base.J
        B = param.base.B
        l = param.base.l
        g = param.base.g
        Kp = param.Kp
        Kd = param.Kd
        alpha = param.base.alpha
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
        dOmegaD = (-(B + Kd) * omegaD - Kp * (thetaD - w)) / J

        DebugDataLogger.PushData(w, "quasi-state")
        DebugDataLogger.PushData(thetaD, "thetaD")

        # saturation
        wSat = util_math.InvertibleSat(w, param.wMin, param.wMax, alpha)
        DebugDataLogger.PushData(wSat, "quasi-stateSat")

        # N
        dThetaN = omegaN
        dOmegaN = (-(B + Kd) * omegaN - Kp * (thetaN - wSat)) / J

        # store quasi-state
        self.w[0] = wSat

        return np.concatenate((np.array([dThetaN, dOmegaN, dThetaD, dOmegaD]), dDeltaDState))
    
    def OutputEquation(self, curState, curInput, param):
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
        tauMin = param.base.tauMin
        tauMax = param.base.tauMax

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

    def ComputeDeltaD(self, curState, curQuasiState, param: Param) -> np.ndarray:
        """
        Compute the state equation of ΔD
        formula : ΔD = σ_u^-1 D σ_w - D

        return:
            np.ndarray: derivative of the state
            float: output of ΔD
        """

        m = param.base.m
        J = param.base.J
        B = param.base.B
        l = param.base.l
        g = param.base.g
        Kp = param.Kp
        Kd = param.Kd
        alpha = param.base.alpha

        thetaD1 = curState[0]
        omegaD1 = curState[1]
        thetaD2 = curState[2]
        omegaD2 = curState[3]
        w = curQuasiState[0]

        # D
        u1 = Kp * (thetaD1 - w) - Kd * omegaD1 - m * g * l * np.sin(thetaD1)

        # σ_u^-1 D σ_w
        wSat = util_math.InvertibleSat(w, param.wMin, param.wMax, alpha)
        u2Sat = Kp * (thetaD2 - wSat) - Kd * omegaD2 - m * g * l * np.sin(thetaD2)
        dThetaD2 = omegaD2
        dOmegaD2 = (-(B + Kd) * omegaD2 - Kp * (thetaD2 - wSat)) / J
        u2 = util_math.InvertibleSatInv(u1, param.base.tauMin, param.base.tauMax, alpha)

        DebugDataLogger.PushData(u2, "tau2DeltaD")

        u = u2 - u1

        return np.array([dThetaD2, dOmegaD2]), u

# ----------------------------------------------------------------------------
# * @file EquivalentSaturateModel.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
