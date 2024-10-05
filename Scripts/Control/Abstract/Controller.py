#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Controller.py
# * @brief Controller class
# * @author hoshina
# * @date 2024/07/02
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from abc import ABC, abstractmethod

from Lib.Utils.DataLogger import DataLogger
from Lib.Utils.GraphPlotter import GraphPlotter

class Controller(ABC):
    """
    Controller class

    Constructor:
        Controller(controllerParam)

    Methods:
        ComputeControlInput: Compute control input
            controlInput = ComputeControlInput(refState, curState, prevSatInput, dt)
    """

    class Param():
        """
        Super class of the controller parameters
        """
        pass

    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): controller parameters
        """
        self.param_ = controllerParam

    def ComputeControlInput(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float) -> np.ndarray:
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
        tmpRefState = refState
        tmpCurState = curState
        tmpPrevSatInput = prevSatInput

        if not isinstance(refState, np.ndarray):
            tmpRefState = np.array([refState])
        if not isinstance(curState, np.ndarray):
            tmpCurState = np.array([curState])
        if not isinstance(prevSatInput, np.ndarray):
            tmpPrevSatInput = np.array([prevSatInput])

        controlInput = self.ComputeControlInputImpl(tmpRefState, tmpCurState, tmpPrevSatInput, dt, self.param_)
        if not isinstance(controlInput, np.ndarray):
            controlInput = np.array([controlInput])

        return controlInput
    
    def PushStateToLogger(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the data logger

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            dataLogger (DataLogger): data logger
        """
        tmpRefState = refState
        tmpCurState = curState
        if not isinstance(refState, np.ndarray):
            tmpRefState = np.array([refState])
        if not isinstance(curState, np.ndarray):
            tmpCurState = np.array([curState])
        self.PushStateToLoggerImpl(tmpRefState, tmpCurState, dataLogger)

    def PushStateForPlot(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the data to the graph plotter

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            graphPlotter (GraphPlotter): graph plotter
        """
        tmpRefState = refState
        if not isinstance(refState, np.ndarray):
            tmpRefState = np.array([refState])
        tmpCurState = curState
        if not isinstance(curState, np.ndarray):
            tmpCurState = np.array([curState])
        self.PushStateForPlotImpl(tmpRefState, tmpCurState, graphPlotter)

    # private ------------------------------------------------------
    @abstractmethod
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
        pass

    @abstractmethod
    def PushStateToLoggerImpl(self, refState: np.ndarray, curState: np.ndarray, dataLogger: DataLogger) -> None:
        """
        Push the state to the data logger

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            dataLogger (DataLogger): data logger
        """
        pass

    @abstractmethod
    def PushStateForPlotImpl(self, refState: np.ndarray, curState: np.ndarray, graphPlotter: GraphPlotter) -> None:
        """
        Push the data to the graph plotter

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            graphPlotter (GraphPlotter): graph plotter
        """
        pass

# ----------------------------------------------------------------------------
# * @file Controller.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
