#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Identificator.py
# * @brief Super class of system identification
# * @author hoshina
# * @date 2024/08/03
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from abc import ABC, abstractmethod

class Identificator(ABC):
    """
    Super class of system identification

    Constructor:
        Identificator(param)

    Methods:
        IdentifySystemModel: Identify the system
            IdentifySystemModel(curInputs, curOutputs, dt)
        UpdateEstimatedOutput: Update the estimated output
            UpdateEstimatedOutput(curInputs, dt)
        GetEstimatedOutput: Get the estimated output
            GetEstimatedOutput()
        ResetOutput: Reset the estimated output
            ResetOutput()
    """

    class Param:
        """
        Parameters of system identification
        """
        def __init__(self, outputOrder: int) -> None:
            """
            constructor
            """
            self.outputOrder = outputOrder
            

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of system identification
        """
        self.param_ = param
        self.estimatedOutput_ = np.zeros(param.outputOrder)

    def IdentifySystemModel(self, curInputs: np.array, curOutputs: np.array, dt: float) -> None:
        """
        Identify the system

        Args:
            curInputs (np.array): input signals in the current step
            curOutputs (np.array): output signals in the current step
            dt (float): time step
        """
        tmpInputs = curInputs
        if not isinstance(curInputs, np.ndarray):
            tmpInputs = np.array([curInputs])
        tmpOutputs = curOutputs
        if not isinstance(curOutputs, np.ndarray):
            tmpOutputs = np.array([curOutputs])

        self.IdentifySystemModelImpl(tmpInputs, tmpOutputs, dt)

    def UpdateEstimatedOutput(self, curInputs: np.array, dt: float) -> None:
        """
        Update the estimated output

        Args:
            curInputs (np.array): input signals in the current step
            dt (float): time step
        """
        tmpInputs = curInputs
        if not isinstance(curInputs, np.ndarray):
            tmpInputs = np.array([curInputs])

        return self.UpdateEstimatedOutputImpl(tmpInputs, dt)
    
    def GetEstimatedOutput(self) -> np.array:
        """
        Get the estimated output

        Returns:
            np.array: estimated output
        """
        return self.estimatedOutput_
    
    def ResetOutput(self) -> None:
        """
        Reset the estimated output
        """
        self.estimatedOutput_ = np.zeros(self.param_.outputOrder)
        self.ResetOutputImpl()
    
    # private -----------------------------------------------------
    @abstractmethod
    def IdentifySystemModelImpl(self, curInputs: np.array, curOutputs: np.array, dt: float) -> None:
        """
        Identify the system

        Args:
            curInputs (np.array): input signals in the current step
            curOutputs (np.array): output signals in the current step
            dt (float): time step
        """
        pass

    @abstractmethod
    def UpdateEstimatedOutputImpl(self, curInputs: np.array, dt: float) -> None:
        """
        Update the estimated output

        Args:
            curInputs (np.array): input signals in the current step
        """
        pass

    @abstractmethod
    def ResetOutputImpl(self) -> None:
        """
        Reset the estimated output
        """
        pass

# ----------------------------------------------------------------------------
# * @file Identificator.py
# * History
# * -------
# * - 2024/08/03 New created.(By hoshina)
