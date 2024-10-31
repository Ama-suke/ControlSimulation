#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file StateObserver.py
# * @brief State observer of mass-spring-damper system
# * @author hoshina
# * @date 2024/10/31
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Controller import Controller

from enum import Enum

import numpy as np
import json

from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter
from DebugDataLogger import DebugDataLogger
from Lib.Compensator.StateSpace import StateSpace
from Control.SeminarFourth.Plant.MassSpringDamper import MassSpringDamper

class StateObserver(Controller):
    """
    controller class

    Constructor:
        StateObserver(controllerParam)

    Methods:
        See Controller class
    """

    class Type(Enum):
        """
        Controller type
        """
        CODIMENTAL = 0              # codimental observer
        MINIMUM_ORDER = 1           # minimum order observer
        MODERN_DISTURBANCE = 2      # disturbance observer based on codimental observer
        CLASSICAL_DISTURBANCE = 3   # disturbance observer based on classical control theory

    class Param(Controller.Param):
        """
        Parameters of controller
        """
        def __init__(self, plantParam: MassSpringDamper.Param, observerGains: np.ndarray, type) -> None:
            """
            constructor

            Args:
                plantParam (MassSpringDamper.Param): plant parameters
                observerGains (np.ndarray): observer gain
                type (StateObserver.Type): type of the observer

            """
            self.plantParam = plantParam
            self.observerGains = observerGains
            self.type = type

        def __str__(self) -> str:
            return json.dumps({
                "StateObserver": {
                    "plantParam": json.loads(str(self.plantParam)),
                    "observerGains": self.observerGains.tolist(),
                    "type": self.type.name
                }})

    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): controller parameters
        """
        super().__init__(controllerParam)

        # Initialize observer
        self.estimatedDisturbance_ = 0
        self.estimatedState_ = np.zeros(2)
        if controllerParam.type == StateObserver.Type.CODIMENTAL:
            self.observer_ = StateSpace(self.CodimentalObserverStateEquation, self.CodimentalObserverOutputEquation)
            self.observerState_ = np.zeros(2)
        elif controllerParam.type == StateObserver.Type.MINIMUM_ORDER:
            self.observer_ = StateSpace(self.MinimumOrderObserverStateEquation, self.MinimumOrderObserverOutputEquation)
            self.observerState_ = np.zeros(1)
        elif controllerParam.type == StateObserver.Type.MODERN_DISTURBANCE:
            self.observer_ = StateSpace(self.ModernDobStateEquation, self.ModernDobOutputEquation)
            self.observerState_ = np.zeros(3)
        elif controllerParam.type == StateObserver.Type.CLASSICAL_DISTURBANCE:
            self.observer_ = StateSpace(self.ClassicalDobStateEquation, self.ClassicalDobOutputEquation)
            self.observerState_ = np.zeros(4)

    # private ------------------------------------------------------
    def ComputeControlInputImpl(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute control input

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current sensing state
            prevSatInput (np.ndarray): previous saturated input
            dt (float): time step
            param (Param):  controller parameters

        Returns:
            np.ndarray: control input
        """

        # input is passed through added disturbance
        controlInput = refState[0] + self.estimatedDisturbance_

        # state observer
        curInputOutput = np.array([controlInput, curState[0]])
        self.estimatedState_ = self.observer_.ComputeOutput(self.observerState_, curInputOutput, param)
        self.observerState_ = self.observer_.ComputeNextState(self.observerState_, curInputOutput, dt, param)

        return np.array([controlInput])

    def PushStateToLoggerImpl(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current sensing state
            dataLogger (DataLogger): data logger
        """
        dataLogger.PushData(self.estimatedState_[0], "EstPos")
        dataLogger.PushData(self.estimatedState_[1], "EstVel")
        dataLogger.PushData(self.estimatedDisturbance_, "EstDist")

    def PushStateForPlotImpl(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the state to the plotter

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current sensing state
            graphPlotter (GraphPlotter): graph plotter
        """
        graphPlotter.PushPlotYData(self.estimatedState_[0], "Estimated", "Position")
        graphPlotter.PushPlotYData(self.estimatedState_[1], "Estimated", "Velocity")
        graphPlotter.PushPlotYData(self.estimatedDisturbance_, "Estimated", "Force")

    # observers ------------------------------------------------------
    def CodimentalObserverStateEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Codimental observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.ndarray: difference of the state
        """
        p = param
        pp = param.plantParam

        curInput = curInputOutput[0]
        curOutput = curInputOutput[1]

        outputError = curOutput - curState[0]

        diffState = np.zeros_like(curState) 

        diffState[0] = curState[1] + p.observerGains[0] * outputError
        diffState[1] = (-pp.springCoef * curState[0] - pp.viscousCoef * curState[1] + curInput) / pp.mass \
                       + p.observerGains[1] * outputError
        
        return diffState
    
    def CodimentalObserverOutputEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Output equation of the codimental observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            param (Param): controller parameters

        Returns:
            np.ndarray: output of the observer
        """
        return curState
    
    def MinimumOrderObserverStateEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Minimum order observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.ndarray: difference of the state
        """
        p = param
        pp = param.plantParam

        a11 = 0
        a12 = 1
        a21 = -pp.springCoef / pp.mass
        a22 = -pp.viscousCoef / pp.mass
        b1 = 0
        b2 = 1 / pp.mass
        vTilde = p.observerGains[0]

        curInput = curInputOutput[0]
        curOutput = curInputOutput[1]

        diffState = np.zeros_like(curState) 

        diffState[0] = (a21 + vTilde * a11 - (a22 + vTilde * a12) * vTilde) * curOutput \
                       + (a22 + vTilde * a12) * curState[0] \
                       + (b2 + vTilde * b1) * curInput
        
        return diffState
    
    def MinimumOrderObserverOutputEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Output equation of the minimum order observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            param (Param): controller parameters

        Returns:
            np.ndarray: output of the observer
        """
        p = param

        curInput = curInputOutput[0]
        curOutput = curInputOutput[1]

        estimatedState = np.zeros(2)
        estimatedState[0] = curOutput
        estimatedState[1] = curState[0] - p.observerGains[0] * curOutput

        return estimatedState
    
    def ModernDobStateEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Modern disturbance observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.ndarray: difference of the state
        """
        p = param
        pp = param.plantParam

        curInput = curInputOutput[0]
        curOutput = curInputOutput[1]

        outputError = curOutput - curState[0]

        diffState = np.zeros_like(curState) 

        diffState[0] = curState[1] + p.observerGains[0] * outputError
        diffState[1] = (-pp.springCoef * curState[0] - pp.viscousCoef * curState[1] + curInput - curState[2]) / pp.mass \
                       + p.observerGains[1] * outputError
        diffState[2] = p.observerGains[2] * outputError
        
        return diffState
    
    def ModernDobOutputEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Output equation of the modern disturbance observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            param (Param): controller parameters

        Returns:
            np.ndarray: output of the observer
        """
        self.estimatedDisturbance_ = curState[2]
        return curState[:2]
    
    def ClassicalDobStateEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Classical disturbance observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.ndarray: difference of the state
        """
        p = param
        pp = param.plantParam

        filterParam = p.observerGains

        curInput = curInputOutput[0]
        curOutput = curInputOutput[1]

        diffState = np.zeros_like(curState) 

        # F(u)
        diffState[0] = curState[1]
        diffState[1] = -filterParam[0] * curState[1] - filterParam[1] * curState[0] + filterParam[1] * curInput
        
        # F(y)
        diffState[2] = curState[3]
        diffState[3] = -filterParam[0] * curState[3] - filterParam[1] * curState[2] + filterParam[1] * curOutput
        
        return diffState
    
    def ClassicalDobOutputEquation(self, curState: np.ndarray, curInputOutput: np.ndarray, param: Param) -> np.ndarray:
        """
        Output equation of the classical disturbance observer

        Args:
            curState (np.ndarray): current state
            curInputOutput (np.ndarray): [current input, current output]
            param (Param): controller parameters

        Returns:
            np.ndarray: output of the observer
        """
        p = param
        pp = param.plantParam

        # u_u
        filteredInput = curState[0]
        # u_y
        x = curState[2]
        dx = curState[3]
        ddx = self.ClassicalDobStateEquation(curState, curInputOutput, param)[3]
        disturbedInput = pp.mass * ddx + pp.viscousCoef * dx + pp.springCoef * x

        self.estimatedDisturbance_ = filteredInput - disturbedInput
        return np.array([x, dx])

# ----------------------------------------------------------------------------
# * @file StateObserver.py
# * History
# * -------
# * - 2024/10/31 New created.(By hoshina)
