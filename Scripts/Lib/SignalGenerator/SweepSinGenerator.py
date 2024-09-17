#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file SweepSinGenerator.py
# * @brief Sweep sine signal generator
# * @author hoshina
# * @date 2024/09/18
# * @details 
# *
# ----------------------------------------------------------------------------

from Lib.SignalGenerator.SignalGenerator import SignalGenerator

import numpy as np
import json

class SweepSinGenerator(SignalGenerator):
    """
    Signal generator class

    Constructor:
        SweepSinGenerator(param)

    Methods:
        GenerateSignal: Generate signal
            signal = GenerateSignal(k, dt)
    """

    class Param(SignalGenerator.Param):
        """
        Parameters of signal generator
        """
        def __init__(self, amplitude, startFreq, stopFreq, finishTime) -> None:
            """
            constructor

            Args:
                amplitude (float): amplitude
                startFreq (float): start frequency
                stopFreq (float): stop frequency
                finishTime (float): finish time

            """
            self.amplitude = amplitude
            self.startFreq = startFreq
            self.stopFreq = stopFreq
            self.finishTime = finishTime
            self.freqStep = (stopFreq - startFreq) * (1 / finishTime)

        def __str__(self) -> str:
            return json.dumps({
                "SweepSinGenerator": {
                    "amplitude": self.amplitude,
                    "startFreq": self.startFreq,
                    "stopFreq": self.stopFreq
                }})

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            signalGeneratorParam (Param): signal generator parameters
        """
        super().__init__(param)

        self.phase_ = 0
        self.freq_ = param.startFreq

    # private ------------------------------------------------------
    def GenerateSignalImpl(self, k: int, dt: float, param: Param) -> float:
        """
        Generate signal

        Args:
            k (int): current discreat time
            dt (float): time step
            param (Param): signal generator parameters

        Returns:
            np.ndarray: signal
        """
        signal = param.amplitude * np.sin(self.phase_)
        self.phase_ += self.freq_ * dt
        self.freq_ += param.freqStep * dt
        return signal

# ----------------------------------------------------------------------------
# * @file SweepSinGenerator.py
# * History
# * -------
# * - 2024/09/18 New created.(By hoshina)
