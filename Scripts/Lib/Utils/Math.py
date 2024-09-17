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
        u0 = util_math.InvertibleSatInvImpl_(y0, uMin, uMax, alpha)
        return util_math.InvertibleSatImpl_(u - u0, uMin, uMax, alpha) + y0
    
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
        u0 = util_math.InvertibleSatInvImpl_(y0, uMin, uMax, alpha)
        return util_math.InvertibleSatInvImpl_(u - y0, uMin, uMax, alpha) + u0
    
    @staticmethod
    def Sat(u: float, uMin: float, uMax: float) -> float:
        """
        Compute the saturation function

        Args:
            u (float): input
            uMin (float): inf(u)
            uMax (float): sup(u)

        Returns:
            float: constrained input
        """
        return np.clip(u, uMin, uMax)
    
    @staticmethod
    def ContinuousSign(u: float, theta: float, gamma: float) -> float:
        """
        Compute the continuous sign function

        Args:
            u (float): input
            theta (float): threshold
            gamma (float): shape parameter

        Returns:
            float: sign
        """
        return np.tanh(np.abs(u / theta) ** gamma) * np.sign(u)
    
    # private ---------------------------------------------------------------
    @staticmethod
    def InvertibleSatImpl_(u: float, uMin: float, uMax: float, alpha: float) -> float:
        g = (uMax - uMin) / 2
        x = np.abs(u / g) ** (1/alpha)
        
        return g * np.tanh(x) ** alpha * np.sign(u)
    
    @staticmethod
    def InvertibleSatInvImpl_(u: float, uMin: float, uMax: float, alpha: float) -> float:
        g = (uMax - uMin) / 2
        x = np.abs(u / g) ** (1/alpha)
        
        return g * np.arctanh(x) ** alpha * np.sign(u)
    
if __name__ == "__main__":
    from DataLogger import DataLogger

    dl = DataLogger()
    for u in np.arange(-0.1, 0.1, 0.001):
        dl.PushData(u, "u")

        gamma = 5
        for theta in np.arange(0.01, 0.1, 0.01):
            y = util_math.ContinuousSign(u, theta, gamma)
            dl.PushData(y, f"y(theta={theta} gamma={gamma})")

        theta = 0.05
        for gamma in np.arange(2, 12, 1):
            y = util_math.ContinuousSign(u, theta, gamma)
            dl.PushData(y, f"y(gamma={gamma} theta={theta})")

    dl.SaveLoggedData("Sign.csv")

# ----------------------------------------------------------------------------
# * @file Math.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
