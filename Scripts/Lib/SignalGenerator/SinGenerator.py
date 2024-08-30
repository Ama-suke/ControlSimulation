#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file SinGenerator.Py
# * @brief Sine signal generator
# * @author hoshina
# * @date 2024/08/30
# * @details 
# *
# ----------------------------------------------------------------------------


import numpy as np
import json
from Lib.SignalGenerator.SignalGenerator import SignalGenerator

class SinGenerator(SignalGenerator):
    """
    Sine signal generator
    """

    class Param(SignalGenerator.Param):
        """
        Parameters of impulse signal generator
        """
        def __init__(self, amplitude: float, frequency: float, phase: float) -> None:
            """
            constructor

            Args:
                amplitude (float): amplitude of sine wave
                frequency (float): frequency of sine wave (Hz)
                phase (float): phase of sine wave (rad)
            """
            self.amplitude = amplitude
            self.frequency = frequency
            self.phase = phase

        def __str__(self) -> str:
            return json.dumps({
                "ImpulseGenerator": {
                    "amplitude": self.amplitude,
                    "frequency": self.frequency,
                    "phase": self.phase
                }})

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of impulse signal generator
        """
        super().__init__(param)

    def GenerateSignalImpl(self, k: int, dt: float, param: Param) -> float:
        return param.amplitude * np.sin(2 * np.pi * param.frequency * dt * k + param.phase)


# ----------------------------------------------------------------------------
# * @file SinGenerator.Py
# * History
# * -------
# * - 2024/08/30 New created.(By hoshina)
