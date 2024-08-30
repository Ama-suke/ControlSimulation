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
import json
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

        def __str__(self) -> str:
            return json.dumps({
                "StepGenerator": {
                    "stepValue": self.stepValue,
                    "initialValue": self.initialValue,
                    "startTimeStep": self.startTimeStep
                }})

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of step signal generator
        """
        super().__init__(param)

    def GenerateSignalImpl(self, k: int, dt: float, param: Param) -> float:
        if k >= param.startTimeStep:
            return param.stepValue
        else:
            return param.initialValue
        

# ----------------------------------------------------------------------------
# * @file StepGenerator.py
# * History
# * -------
# * - 2024/08/22 New created.(By hoshina)
