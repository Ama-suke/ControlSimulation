#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * @brief Operator controller class
# * @author hoshina
# * @date 2024/08/29
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
import json

from Control.Abstract.Controller import Controller
from Control.Pendulum.Plant.Pendulum import Pendulum
from Lib.Utils.StateSpace import StateSpace
from Lib.Utils.Math import util_math

from DebugDataLogger import DebugDataLogger

class OperatorController(Controller):
    """
    Operator controller class

    Constructor:
        OperatorController(controllerParam)

    Methods:
        ComputeControlInput: Compute control input
            controlInput = ComputeControlInput(refState, curState, prevSatInput, dt)
    """

    class Param(Controller.Param):
        """
        Parameters of operator controller
        """
        def __init__(self, Kp, Kd, tau, gF, plantParam: Pendulum.Param) -> None:
            """
            constructor

            Args:
                Kp (float): proportional gain
                Kd (float): derivative gain
                tau (float): time constant of the low-pass filter
                gF (float): gain of friction compensation
                plantParam (Pendulum.Param): parameters of the plant
            """
            self.Kp = Kp
            self.Kd = Kd
            self.tau = tau
            self.gF = gF
            self.plantParam = plantParam

        def __str__(self) -> str:
            return json.dumps({
                "OperatorController": {
                    "Kp": self.Kp,
                    "Kd": self.Kd,
                    "tau": self.tau
                }})

    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): parameters of operator controller
        """
        super().__init__(controllerParam)
        self.stateSpace_ = StateSpace(self.FirstOrderStateEquation)
        self.yLpf_ = 0
        self.refState_ = np.zeros(2)

    # private ----------------------------------------------------------------
    def ComputeControlInputImpl(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute control input

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            prevSatInput (np.ndarray): previous saturated input
            dt (float): time step

        Returns:
            np.ndarray: control input
        """
        p = param
        pp = p.plantParam

        ref = refState[1] - refState[0]
        theta = curState[0]

        self.yLpf_ = self.stateSpace_.ComputeNextState(self.yLpf_,         theta, dt, p)
        dy = self.FirstOrderStateEquation(self.yLpf_, theta, p)

        DebugDataLogger.PushData(dy, "dy")

        pdFeedback = p.Kp * (ref - theta) - p.Kd * dy
        gravityTorque = - pp.m * pp.g * pp.l * np.sin(theta)
        sign_pdFeedback = util_math.InvertibleSat(1000 * pdFeedback, -1, 1, 0.1)
        frictionTorque = p.gF * pp.f0 * np.exp(1 - pp.k * np.abs(dy)) * sign_pdFeedback
        ffTorque = self.ComputeFeedForwardTorque(refState, dt, p)
        
        DebugDataLogger.PushData(pdFeedback, "pdFeedback")
        DebugDataLogger.PushData(gravityTorque, "gravityTorque")
        DebugDataLogger.PushData(frictionTorque, "frictionTorque")
        DebugDataLogger.PushData(ffTorque, "ffTorque")

        controlInput = pdFeedback + gravityTorque + frictionTorque + ffTorque

        satControlInput = util_math.InvertibleSat(controlInput, pp.tauMin, pp.tauMax, pp.alpha)

        return satControlInput
    
    def PushStateToLoggerImpl(self, refState: np.ndarray, dataLogger) -> None:
        """
        Push the state to the data logger

        Args:
            controlInput (np.ndarray): control input
            dataLogger: data logger
        """
        dataLogger.PushData(refState[1] - refState[0], "Ref")

    def FirstOrderStateEquation(self, curState, curInput, param: Param):
        """
        State equation of first order system

        Args:
            curState (np.array): current state
            curInput (np.array): current input
            param (Param): parameters

        Returns:
            np.array: state
        """
        return (-curState + curInput) / param.tau
    
    def ComputeFeedForwardTorque(self, refState: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute feed forward torque

        Args:
            refState (np.ndarray): reference state
            dt (float): time step
            param (Param): parameters of the controller

        Returns:
            np.ndarray: feed forward torque
        """
        p = param
        pp = p.plantParam
        
        curRefState = self.refState_

        # compute reference derivative
        ref = refState[1] - refState[0]

        newRefState = np.zeros(2)
        newRefState[0] = curRefState[1]
        newRefState[1] = ref

        dref = (newRefState[1] - curRefState[1]) / dt
        d2ref = (newRefState[1] - 2 * curRefState[1] + curRefState[0]) / dt ** 2

        DebugDataLogger.PushData(ref, "Ref")
        DebugDataLogger.PushData(dref, "dref")
        DebugDataLogger.PushData(d2ref, "d2ref")
        DebugDataLogger.PushData(curRefState[0], "RefState0")
        DebugDataLogger.PushData(curRefState[1], "RefState1")
        DebugDataLogger.PushData(newRefState[1], "RefState2")

        self.refState_ = newRefState

        u = pp.J * d2ref + pp.D * dref
        return u

# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * History
# * -------
# * - 2024/08/29 New created.(By hoshina)
