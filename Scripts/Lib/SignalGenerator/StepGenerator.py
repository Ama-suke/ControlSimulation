#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file StepGenerator.py
# * @brief Step signal generator
# * @author hoshina
# * @date 2024/08/22
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from Lib.SignalGenerator.SignalGenerator import SignalGenerator

class StepGenerator(SignalGenerator):
    """
    Step signal generator

    Constructor:
        StepGenerator(param)

    Methods:
        GenerateSignal: Generate a signal
            GenerateSignal(t, dt)
    """

    class Param(SignalGenerator.Param):
        """
        Parameters of step signal generator
        """
        def __init__(self, stepValue: float, initialValue: float, startTimeStep: int) -> None:
            """
            constructor

            Args:
                stepValue (float): step value
                initialValue (float): initial value
                startTimeStep (int): start time step
            """
            self.stepValue = stepValue
            self.initialValue = initialValue
            self.startTimeStep = startTimeStep

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of step signal generator
        """
        super().__init__(param)

    def GenerateSignalImpl(self, k: int, dt: float) -> float:
        """
        Generate a signal

        Args:
            k (int): discrete time
            dt (float): sampling time

        Returns:
            float: signal
        """
        if k >= self.param_.startTimeStep:
            return self.param_.stepValue
        else:
            return self.param_.initialValue
        

# ----------------------------------------------------------------------------
# * @file StepGenerator.py
# * History
# * -------
# * - 2024/08/22 New created.(By hoshina)
