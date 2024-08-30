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
    def InvertibleSat(u: float, uMin: float, uMax: float, alpha: float) -> float:
        """
        Compute the invertible saturation function

        Args:
            u (float): input
            uMin (float): inf(u)
            uMax (float): sup(u)
            alpha (float): shape parameter (0 < alpha < 1)

        Returns:
            float: constrained input
        """
        y0 = (uMax + uMin) / 2
        u0 = util_math.ComputeInvertibleSatInvImpl_(y0, uMin, uMax, alpha)
        return util_math.ComputeInvertibleSatImpl_(u - u0, uMin, uMax, alpha) + y0
    
    @staticmethod
    def InvertibleSatInv(u: float, uMin: float, uMax: float, alpha: float) -> float:
        """
        Compute the inverse of the invertible saturation function

        Args:
            u (float): constrained input
            uMin (float): inf(u)
            uMax (float): sup(u)
            alpha (float): shape parameter (0 < alpha < 1)

        Returns:
            float: input
        """
        y0 = (uMax + uMin) / 2
        u0 = util_math.ComputeInvertibleSatInvImpl_(y0, uMin, uMax, alpha)
        return util_math.ComputeInvertibleSatInvImpl_(u - y0, uMin, uMax, alpha) + u0
    
    # private ---------------------------------------------------------------
    @staticmethod
    def ComputeInvertibleSatImpl_(u: float, uMin: float, uMax: float, alpha: float) -> float:
        g = (uMax - uMin) / 2
        x = np.abs(u / g) ** (1/alpha)
        
        return g * np.tanh(x) ** alpha * np.sign(u)
    
    @staticmethod
    def ComputeInvertibleSatInvImpl_(u: float, uMin: float, uMax: float, alpha: float) -> float:
        g = (uMax - uMin) / 2
        x = np.abs(u / g) ** (1/alpha)
        
        return g * np.arctanh(x) ** alpha * np.sign(u)
    
if __name__ == "__main__":
    from DataLogger import DataLogger

    dl = DataLogger()
    for u in np.arange(-10.0, 10.0, 0.1):
        dl.PushData(u, "u")
        for alpha in np.arange(0.1, 1.1, 0.1):
            y = util_math.InvertibleSat(u, -2.0, 5.0, alpha)
            dl.PushData(y, f"y(alpha={alpha})")

    dl.SaveLoggedData("InvertibleSat.csv")

# ----------------------------------------------------------------------------
# * @file Math.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
