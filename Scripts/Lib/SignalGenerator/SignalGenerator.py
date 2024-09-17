#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file SignalGenerator.py
# * @brief SignalGenerator abstruct class
# * @author hoshina
# * @date 2024/08/03
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from abc import ABC, abstractmethod

class SignalGenerator(ABC):
    """
    SignalGenerator abstruct class

    Constructor:
        SignalGenerator(param)

    Methods:
        GenerateSignal: Generate a signal
            GenerateSignal(k, dt)
    """

    class Param:
        """
        Parameters of signal generator
        """
        pass


    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of signal generator
        """
        self.param_ = param

    def GenerateSignal(self, k: float, dt: float) -> float:
        """
        Generate a signal

        Args:
            k (int): discrete time
            dt (float): sampling time

        Returns:
            float: signal
        """
        return self.GenerateSignalImpl(k, dt, self.param_)
    
    # private -----------------------------------------------------
    @abstractmethod
    def GenerateSignalImpl(self, k: int, dt: float, param: Param) -> float:
        """
        Generate a signal

        Args:
            k (int): discrete time
            dt (float): sampling time
            param (Param): parameters of signal generator

        Returns:
            float: signal
        """
        pass



# ----------------------------------------------------------------------------
# * @file SignalGenerator.py
# * History
# * -------
# * - 2024/08/03 New created.(By hoshina)
