#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Math.py
# * @brief Additional math functions
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np

# namespace util_math
class util_math:

    @staticmethod
    def ComputeInvertibleSat(u: float, uMin: float, uMax: float) -> float:
        """
        Compute the invertible saturation function

        Args:
            u (float): input
            wMin (float): inf(u)
            wMax (float): sup(u)

        Returns:
            float: quasi-state
        """
        return (uMax - uMin) / 2 * np.tanh(u) + (uMax + uMin) / 2
    
    @staticmethod
    def ComputeInvertibleSatInv(w: float, uMin: float, uMax: float) -> float:
        """
        Compute the inverse of the invertible saturation function

        Args:
            w (float): quasi-state
            wMin (float): inf(u)
            wMax (float): sup(u)

        Returns:
            float: input
        """
        return np.arctanh(2 * (w - (uMax + uMin) / 2) / (uMax - uMin))

# ----------------------------------------------------------------------------
# * @file Math.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
