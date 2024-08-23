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

    def ComputeControlInput(self, refState: np.array, curState: np.array, prevSatInput: np.array, dt: float) -> np.array:
        """
        Compute control input

        Args:
            refState (np.array): reference state
            curState (np.array): current state
            prevSatInput (np.array): previous saturated input
            dt (float): time step

        Returns:
            np.array: control input
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
    
    def PushStateToLogger(self, refState: np.array, dataLogger: DataLogger) -> None:
        """
        Push the state to the data logger

        Args:
            controlInput (np.array): control input
            dataLogger (DataLogger): data logger
        """
        tmpRefState = refState
        if not isinstance(refState, np.ndarray):
            tmpRefState = np.array([refState])
        self.PushStateToLoggerImpl(tmpRefState, dataLogger)

    # private ------------------------------------------------------
    @abstractmethod
    def ComputeControlInputImpl(self, refState: np.array, curState: np.array, prevSatInput: np.array, dt: float, param: Param) -> np.array:
        """
        Compute control input

        Args:
            refState (np.array): reference state
            curState (np.array): current state
            prevSatInput (np.array): previous saturated input
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.array: control input
        """
        pass

    @abstractmethod
    def PushStateToLoggerImpl(self, refState: np.array, dataLogger: DataLogger) -> None:
        """
        Push the state to the data logger

        Args:
            controlInput (np.array): control input
            dataLogger (DataLogger): data logger
        """
        pass

# ----------------------------------------------------------------------------
# * @file Controller.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)
