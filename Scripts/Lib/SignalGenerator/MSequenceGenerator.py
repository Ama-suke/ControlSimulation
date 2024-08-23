#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file MSequenceGenerator.py
# * @brief M-sequence generator
# * @author hoshina
# * @date 2024/08/03
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from Lib.SignalGenerator.SignalGenerator import SignalGenerator

class MSequenceGenerator(SignalGenerator):
    """
    M-sequence generator

    Constructor:
        MSequenceGenerator(param)

    Methods:
        GenerateSignal: Generate M-sequence signal
            GenerateSignal(i, dt)
    """

    class Param(SignalGenerator.Param):
        """
        Parameters of M-sequence generator
        """
        def __init__(self, order: int, amplitude: float, offset: float) -> None:
            """
            constructor

            Args:
                mSeqOrder (int): order of M-sequence
                amplitude (float): amplitude
                offset (float): offset

            description:
                M-sequence is a pseudo-random binary sequence
                lower value "low" is
                    low = -amplitude + offset
                higher value "high" is
                    high = amplitude + offset
            """
            self.order = order
            self.amplitude = amplitude
            self.offset = offset

    def __init__(self, param: Param) -> None:
        self.param_ = param
        self.mSeq_ = self.GenerateMSeq(self.param_.order)

    def GenerateSignalImpl(self, k: int, dt: float) -> np.array:
        """
        Generate M-sequence signal

        Args:
            k (int): current step
            dt (float): time step

        Returns:
            np.array: generated signal
        """
        p = self.param_

        sig = 2 * p.amplitude * (np.array(self.mSeq_[k % len(self.mSeq_)]) - 0.5) + p.offset
        return np.array([sig])

    def GenerateMSeq(self, mSeqOrder: int) -> np.array:
        """
        Generate M-sequence

        Args:
            mSeqOrder (int): order of M-sequence

        Returns:
            np.array: generated M-sequence
        """
        reg = [1] * mSeqOrder  # Example initialization
        mSeq = []
        for _ in range(2**mSeqOrder - 1):
            newBit = int(reg[0]) ^ int(reg[-1])  # Ensure elements are integers
            mSeq.append(reg.pop())
            reg.insert(0, newBit)
        return mSeq
# ----------------------------------------------------------------------------
# * @file MSequenceGenerator.py
# * History
# * -------
# * - 2024/08/03 New created.(By hoshina)
