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
from enum import Enum

from Control.Abstract.Controller import Controller
from Control.SimplePendulum.Plant.Pendulum import Pendulum
from Lib.Compensator.StateSpace import StateSpace
from Lib.Utils.Math import util_math
from Lib.Compensator.FunnelControl import FunnelControl
from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter
from Lib.Compensator.PidServo import PidServo

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

    class QOperator(Enum):
        """
        Q operator
        """
        PD = 0
        FUNNEL = 1

    class Param(Controller.Param):
        """
        Parameters of operator controller
        """
        def __init__(self, pidParam: PidServo.Param, tauD, pR: np.ndarray, gF, theta, gamma, xDeltaBar, funnelParam: FunnelControl.Param, boundaryFunction: FunnelControl.BoundaryFunction, plantParam: Pendulum.Param) -> None:
            """
            constructor

            Args:
                pidParam (PidServo.Param): parameters of the PID controller
                tauD (float): time constant of the low-pass filter
                pR (np.ndarray): parameters of the second order low-pass filter
                gF (float): gain of friction compensation
                theta (float): shape parameter of the sign function
                gamma (float): shape parameter of the sign function
                xDeltaBar (float): upper bounds of disturbance
                funnelParam (FunnelControl.Param): parameters of the funnel control
                plantParam (Pendulum.Param): parameters of the plant
                qType (QOperator): type of Q operator
            """
            self.pidParam = pidParam
            self.tauD = tauD
            self.pR = pR
            self.gF = gF
            self.theta = theta
            self.gamma = gamma
            self.xDeltaBar = xDeltaBar
            self.funnelParam = funnelParam
            self.boundaryFunction = boundaryFunction
            self.plantParam = plantParam

        def __str__(self) -> str:
            return json.dumps({
                "OperatorController": {
                    "pidParam": json.loads(str(self.pidParam)),
                    "tauD": self.tauD,
                    "pR": self.pR.tolist(),
                    "gF": self.gF,
                    "theta": self.theta,
                    "gamma": self.gamma,
                    "xDeltaBar": self.xDeltaBar,
                    "funnelParam": json.loads(str(self.funnelParam)),
                    "boundaryFunction": json.loads(str(self.boundaryFunction)),
                    "plantParam": json.loads(str(self.plantParam))
                }})

    def __init__(self, controllerParam: Param, qType: QOperator) -> None:
        """
        constructor

        Args:
            controllerParam (Param): parameters of operator controller
        """
        super().__init__(controllerParam)
        self.firstOrderLpf_ = StateSpace(self.FirstOrderLpfStateEquation)
        self.SecondOrderLpf_ = StateSpace(self.SecondOrderLpfStateEquation)
        self.pdControl_ = PidServo(controllerParam.pidParam)
        self.funnelControl_ = FunnelControl(controllerParam.funnelParam, controllerParam.boundaryFunction)
        self.qType_ = qType

        self.yLpf_ = 0
        self.errorLpf_ = 0
        self.refState_ = np.zeros(2)
        self.bounds_ = np.zeros(2)

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

        ref = refState[0]
        theta = curState[0]

        self.yLpf_ = self.firstOrderLpf_.ComputeNextState(self.yLpf_, theta, dt, p.tauD)
        dy = self.FirstOrderLpfStateEquation(self.yLpf_, theta, p.tauD)

        DebugDataLogger.PushData(dy, "dy")

        qOpTorque = self.ComputeQOperatorTorque(ref, theta, dt, p)
        gravityTorque = - pp.m * pp.g * pp.l * np.sin(theta)
        signQOpTorque = util_math.ContinuousSign(qOpTorque, theta=p.theta, gamma=p.gamma)
        frictionTorque = p.gF * pp.f0 * np.exp(-np.abs(dy) / pp.vSigma) * signQOpTorque
        ffTorque = self.ComputeFeedForwardTorque(ref, dt, p)
        
        DebugDataLogger.PushData(qOpTorque, "qOpTorque")
        DebugDataLogger.PushData(gravityTorque, "gravityTorque")
        DebugDataLogger.PushData(frictionTorque, "frictionTorque")
        DebugDataLogger.PushData(ffTorque, "ffTorque")

        controlInput = (qOpTorque + gravityTorque + frictionTorque + pp.B * dy) / pp.R

        satControlInput = util_math.InvertibleSat(controlInput, pp.tauMin, pp.tauMax, pp.alpha)

        return satControlInput
    
    def PushStateToLoggerImpl(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the data logger

        Args:
            controlInput (np.ndarray): control input
            dataLogger: data logger
        """
        dataLogger.PushData(refState[0], "Ref")
        dataLogger.PushData(refState[0] - curState[0], "Error")
        dataLogger.PushData(self.bounds_[0], "UpperBounds")
        dataLogger.PushData(-self.bounds_[0], "LowerBounds")

    def PushStateForPlotImpl(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the data to the graph plotter

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PushPlotYData(refState[0], "Ref", "Position")
        graphPlotter.PushPlotYData(refState[0] - curState[0], "Error", "Error")
        graphPlotter.PushPlotYData(self.bounds_[0], "UpperBounds", "Error")
        graphPlotter.PushPlotYData(-self.bounds_[0], "LowerBounds", "Error")

    def FirstOrderLpfStateEquation(self, curState, curInput, tau):
        """
        State equation of first order system

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): parameters

        Returns:
            np.ndarray: state
        """
        return (-curState + curInput) / tau
    
    def SecondOrderLpfStateEquation(self, curState, curInput, p: np.ndarray):
        """
        State equation of second order system

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            p (np.ndarray): filter parameters

        Returns:
            np.ndarray: state
        """
        diffState = np.zeros(2)
        diffState[0] = curState[1]
        diffState[1] = (-p[0] * curState[0] - p[1] * curState[1] + p[0] * curInput)

        return diffState

    
    def ComputeFeedForwardTorque(self, refTheta: float, dt: float, param: Param) -> np.ndarray:
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
        dref = curRefState[1]
        d2ref = self.SecondOrderLpfStateEquation(curRefState, refTheta, p.pR)[1]

        u = pp.J * d2ref
        
        self.refState_ = self.SecondOrderLpf_.ComputeNextState(curRefState, refTheta, dt, p.pR)
        return u
    
    def ComputeQOperatorTorque(self, refTheta: float, curTheta: float, dt: float, param: Param) -> np.ndarray:
        """
        Compute Q operator torque

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            param (Param): parameters of the controller

        Returns:
            np.ndarray: Q operator torque
        """
        p = param
        pp = p.plantParam

        theta = curTheta
        
        dy = self.FirstOrderLpfStateEquation(self.yLpf_, theta, p.tauD)

        qOpTorque = self.pdControl_.ComputeControlInput(refTheta, theta, dt)
        if (self.qType_ is OperatorController.QOperator.PD):
            return qOpTorque
        elif (self.qType_ is OperatorController.QOperator.FUNNEL):
            errorStates = self.pdControl_.GetStates()[0:2]
            qOpTorque = self.funnelControl_.ComputeControlInput(errorStates, dt)
            qOpTorque += util_math.ContinuousSign(dy, theta=p.theta, gamma=p.gamma) * p.xDeltaBar

            self.bounds_ = self.funnelControl_.ComputeBoundaryFunction()

            DebugDataLogger.PushData(errorStates[0], "funnelError")
            DebugDataLogger.PushData(errorStates[1], "funnelDError")
            DebugDataLogger.PushData(self.bounds_[0], "funnelUpperBounds")
            DebugDataLogger.PushData(-self.bounds_[0], "funnelLowerBounds")
            DebugDataLogger.PushData(self.bounds_[1], "funnelUpperDBounds")
            DebugDataLogger.PushData(-self.bounds_[1], "funnelLowerDBounds")
            
            return qOpTorque
        else:
            raise ValueError("Invalid Q operator type")


# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * History
# * -------
# * - 2024/08/29 New created.(By hoshina)
