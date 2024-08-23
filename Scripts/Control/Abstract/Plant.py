#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Plant.py
# * @brief Super class of controlled object
# * @author hoshina
# * @date 2024/07/01
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from Lib.Utils.StateSpace import StateSpace
from abc import ABC, abstractmethod

from Lib.Utils.DataLogger import DataLogger

class Plant(ABC):
    """
    Super class of controlled object

    Constructor:
        Plant(stateOrder, plantParam, solverType = StateSpace.SolverType.RUNGE_KUTTA, initialState = None)

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

    class Param():
        """
        Super class of the plant parameters
        """
        pass

    def __init__(self, stateOrder: int, plantParam: Param,
                 solverType: StateSpace.SolverType = StateSpace.SolverType.RUNGE_KUTTA,
                 initialState: np.array = None) -> None:
        """
        constructor

        Args:
            stateOrder (int): state order
            plantParam (Param): plant parameters
            solverType (StateSpace.SolverType, optional): solver type. Defaults to StateSpace.SolverType.RUNGE_KUTTA.
            initialState (np.array, optional): initial state. Defaults to zero.
        """
        self.stateSpace_ = StateSpace(self.StateEquation, solverType)
        
        self.stateVariable_ = np.zeros(stateOrder)
        if initialState is not None:
            if len(initialState) != stateOrder:
                raise ValueError("The length of initialState must be equal to stateOrder.")
            self.stateVariable_ = initialState

        self.plantParam_ = plantParam

    def UpdateState(self, curInput: np.array, dt: float) -> None:
        """
        Update the state of the plant

        Args:
            curInput (np.array): current input
            dt (float): time step
        """
        tmpCurInput = curInput
        if not isinstance(curInput, np.ndarray):
            tmpCurInput = np.array([curInput])

        self.stateVariable_ = self.stateSpace_.ComputeNextState(self.stateVariable_, tmpCurInput, dt, self.plantParam_)
    
    def GetState(self):
        """
        Get the current state

        Returns:
            np.array: current state
        """
        return self.stateVariable_
    
    def GetOutput(self, curInput: np.array) -> np.array:
        """
        Get the output of the plant

        Args:
            curInput (np.array): current input

        Returns:
            np.array: output
        """
        tmpCurInput = curInput
        if not isinstance(curInput, np.ndarray):
            tmpCurInput = np.array([curInput])

        return self.OutputEquation(self.stateVariable_, tmpCurInput, self.plantParam_)
    
    def GetSaturatedInput(self, curInput: np.array) -> np.array:
        """
        Get the saturated input

        Args:
            u (np.array): input

        Returns:
            np.array: saturated input
        """
        input = curInput
        if not isinstance(curInput, np.ndarray):
            input = np.array([curInput])

        return self.GetSaturatedInputImpl(input, self.plantParam_)
    
    def SetState(self, state: np.array) -> None:
        """
        Set the state

        Args:
            state (np.array): state
        """
        self.stateVariable_ = state

    def PushStateToLogger(self, curInput: np.array, logger: DataLogger) -> None:
        """
        Push the state to the logger

        Args:
            logger (DataLogger): logger
        """
        self.PushStateToLoggerImpl(curInput, logger)

    # private ------------------------------------------------------
    @abstractmethod
    def StateEquation(self, curState: np.array, curInput: np.array, param: np.array) -> np.array:
        """
        State equation of the plant.
        Please override this function in the derived class.
        
        Args:
            curState (np.array): current state
            curInput (np.array): current input
            param (np.array): parameters

        Returns:
            np.array: derivative of the state
        """
        pass
    
    @abstractmethod
    def OutputEquation(self, curState: np.array, curInput: np.array, param: np.array) -> np.array:
        """
        Output equation of the plant.
        Please override this function in the derived class.
        
        Args:
            curState (np.array): current state
            curInput (np.array): current input
            param (np.array): parameters

        Returns:
            np.array: output
        """
        pass

    @abstractmethod
    def GetSaturatedInputImpl(self, u: np.array, param: np.array) -> np.array:
        """
        Get the saturated input.
        Please override this function in the derived class.
        
        Args:
            u (np.array): input
            param (np.array): parameters

        Returns:
            np.array: saturated input
        """
        pass

    @abstractmethod
    def PushStateToLoggerImpl(self, logger: DataLogger) -> None:
        """
        Push the state to the logger.
        Please override this function in the derived class.
        
        Args:
            logger (DataLogger): logger
        """
        pass

if __name__ == "__main__":
    pass


# ----------------------------------------------------------------------------
# * @file Plant.py
# * History
# * -------
# * - 2024/07/01 New created.(By hoshina)
