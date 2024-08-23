#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file ImpulseGenerator.py
# * @brief Impulse signal generator
# * @author hoshina
# * @date 2024/08/03
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from Lib.SignalGenerator.SignalGenerator import SignalGenerator

class ImpulseGenerator(SignalGenerator):
    """
    Impulse signal generator

    Constructor:
        ImpulseGenerator(param)

    Methods:
        GenerateSignal: Generate a signal
            GenerateSignal(t, dt)
    """

    class Param(SignalGenerator.Param):
        """
        Parameters of impulse signal generator
        """
        def __init__(self, amplitude: float, startTimeStep: int) -> None:
            """
            constructor

            Args:
                amplitude (float): amplitude of impulse
                startTimeStep (int): start time step
            """
            self.amplitude = amplitude
            self.startTimeStep = startTimeStep

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of impulse signal generator
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
        if k == self.param_.startTimeStep:
            return self.param_.amplitude
        else:
            return 0.0

# ----------------------------------------------------------------------------
# * @file ImpulseGenerator.py
# * History
# * -------
# * - 2024/08/03 New created.(By hoshina)
