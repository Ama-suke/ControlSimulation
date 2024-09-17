#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file MorohoshiController.py
# * @brief Morohoshi controller
# * @author hoshina
# * @date 2024/08/27
# * @details 
# *
# ----------------------------------------------------------------------------


import numpy as np
import json

from Control.Abstract.Controller import Controller
from Lib.Compensator.PidServo import PidServo
from Lib.Utils.DataLogger import DataLogger
from Lib.Compensator.StateSpace import StateSpace
from Control.InvertedWheelPendulum.Plant.InvertedWheelPendulum import InvertedWheelPendulum
from DebugDataLogger import DebugDataLogger

class MorohoshiController(Controller):
    """
    Morohoshi controller class
    """
    class Param(Controller.Param):
        """
        Parameters of Morohoshi controller

        Constructor:
            Param(plantParam, k1, k2, k3, lpfTau)
        """
        def __init__(self, plantParam: InvertedWheelPendulum.Param, k1: float, k2: float, k3: float, lpfTau: float) -> None:
            """
            constructor

            Args:
                plantParam (InvertedWheelPendulum.Param): parameters of inverted wheel pendulum
                k1 (float): k1
                k2 (float): k2
                k3 (float): k3
                lpfTau (float): time constant of low pass filter
                
            """
            self.plantParam = plantParam
            self.k1 = k1
            self.k2 = k2
            self.k3 = k3
            self.lpfTau = lpfTau

        def __str__(self) -> str:
            return json.dumps({
                "MorohoshiController": {
                    "plantParam": json.loads(str(self.plantParam)),
                    "k1": self.k1,
                    "k2": self.k2,
                    "k3": self.k3,
                    "lpfTau": self.lpfTau
                }})
        
    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): controller parameters
        """
        super().__init__(controllerParam)
        self.lpf_ = StateSpace(self.FistOrderLpfStateEquation)
        self.theta_ = 0.0
        self.phi_ = 0.0

    def ComputeControlInputImpl(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute control input

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            prevSatInput (np.ndarray): previous saturated input
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.ndarray: control input
        """
        p = param
        pp = p.plantParam

        theta = curState[0]
        x = curState[1]
        phi = x / pp.r - theta
        ref = refState[0]

        # LPF
        dTheta = -(self.theta_ - theta) / dt
        dPhi = -(self.phi_ - phi) / dt
        self.theta_ = self.lpf_.ComputeNextState(self.theta_, theta, dt, p)
        self.phi_ = self.lpf_.ComputeNextState(self.phi_, phi, dt, p)

        DebugDataLogger.PushData(self.theta_, "MorohoshiController/theta")
        DebugDataLogger.PushData(self.phi_, "MorohoshiController/phi")
        DebugDataLogger.PushData(dTheta, "MorohoshiController/dTheta")
        DebugDataLogger.PushData(dPhi, "MorohoshiController/dPhi")

        # TODO: cは2x1の任意のベクトルなのでパラメータから入れる
        c = np.array([1, 0])
        b = np.array([0, 1])
        
        J = np.array([[pp.J_p + pp.J_w + (pp.m_w + pp.m_p) * pp.r ** 2 + pp.m_p * pp.l_g ** 2 + 2 * pp.m_p * pp.r * pp.l_g * np.cos(theta),
                       pp.J_w + (pp.m_w + pp.m_p) * pp.r ** 2 + pp.m_p * pp.r * pp.l_g * np.cos(theta)],
                      [pp.J_w + (pp.m_w + pp.m_p) * pp.r ** 2 + pp.m_p * pp.r * pp.l_g * np.cos(theta),
                       pp.J_w + (pp.m_w + pp.m_p) * pp.r ** 2]
                       ])
        J_inv = np.linalg.inv(J)
        # fとdは一緒に入れてる
        fd = np.array([-pp.D_p * dTheta + pp.m_p * pp.l_g * (pp.g + pp.r * dTheta ** 2) * np.sin(theta),
                   -pp.D_w * dPhi + pp.m_p * pp.r * pp.l_g * dTheta ** 2 * np.sin(theta)])
        
        states = np.array([theta, phi])
        dStates = np.array([dTheta, dPhi])

        # Compute control input
        controlInput = np.dot(c, 
                              - np.linalg.solve(J, fd)
                              - p.k1 * dStates - p.k2 * states + p.k3 * b * ref)
        controlInput /= np.dot(c, J_inv @ b)

        return np.array([controlInput])

    def PushStateToLoggerImpl(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        dataLogger.PushData(refState[0], "refPhi")

    # private ------------------------------------------------------
    def FistOrderLpfStateEquation(self, curState: float, curInput: float, param: Param) -> float:
        """
        First order low pass filter

        Args:
            curState (float): current state
            curInput (float): current input
            dt (float): time step

        Returns:
            float: differential of the state
        """
        return (curInput - curState) / param.lpfTau

# ----------------------------------------------------------------------------
# * @file MorohoshiController.py
# * History
# * -------
# * - 2024/08/27 New created.(By hoshina)
