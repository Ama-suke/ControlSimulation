#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file FunnelControl.py
# * @brief The class for funnel control
# * @author hoshina
# * @date 2024/09/11
# * @details
# *     Formula:
# *         u(t) = Σi (Ki λi(t)^ci ei(t) / |λi(t)^ci - |ei(t)|^ci|)
# *
# ----------------------------------------------------------------------------

import numpy as np
import json
from enum import Enum
from abc import ABC, abstractmethod

from DebugDataLogger import DebugDataLogger

class FunnelControl:
    """
    The class for funnel control

    Constructor:
        FunnelControl(param, boundaryFunctionType)

    Methods:
        ComputeBoundaryFunction: Compute the boundary function
            lambdaVec = ComputeBoundaryFunction()
        ComputeControlInput: Compute the control input
            controlInput = ComputeControlInput(errorVec, dt)
        ResetTime: Reset the time for the boundary function
            ResetTime()
    """

    class BoundaryFunction(ABC):
        """
        The class of boundary function
        """
        class Type(Enum):
            """
            The type of the boundary function
            """
            EXP = 0
            CSCH = 1
            USER_DEFINED = 2

        def __init__(self, type: Type, order: int) -> None:
            # parameters
            self.type_ = type
            self.order_ = order

            # variables
            self.zeroTime_ = 0

        def Reset(self, currentTime: float):
            """
            Reset boundary function
            """
            self.zeroTime_ = currentTime

        
        def ComputeBounds(self, currentTime: float) -> np.ndarray:
            """
            Compute the bounds

            Args:
                t (float): time

            Returns:
                np.ndarray: the bounds
            """
            return self.ComputeBoundsImpl(currentTime - self.zeroTime_)

        @abstractmethod
        def ComputeBoundsImpl(self, time: float) -> np.ndarray:
            """
            Compute the bounds

            Args:
                t (float): time

            Returns:
                np.ndarray: the bounds
            """
            pass

    class Param:
        def __init__(self, K: np.ndarray, c: np.ndarray):
            """
            Constructor

            Args:
                K (np.ndarray): the gain matrix
                c (np.ndarray): the shape parameter of the control input
            """
            self.K = K
            self.c = c

        def __str__(self):
            """
            Convert to string

            Returns:
                str: the string
            """
            return json.dumps({
                "FunnelControl": {
                    "K": self.K.tolist(),
                    "c": self.c.tolist()
                }}, indent=4)

    def __init__(self, param: Param, boundaryFunction: BoundaryFunction) -> None:
        """
        Constructor

        Args:
            param (Param): parameters
            boundaryFunctionType (BoundaryFunctionType): the boundary function type
        """
        self.boundaryFunction_ = boundaryFunction

        self.param_ = param
        self.order_ = boundaryFunction.order_
        
        self.currentTime_ = 0
        
    def ComputeBoundaryFunction(self) -> np.ndarray:
        """
        Compute the boundary function

        Returns:
            np.ndarray: the boundary function
        """
        return self.boundaryFunction_.ComputeBounds(self.currentTime_)
    
    def ComputeControlInput(self, errorVec: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute the control input

        Args:
            errorVec (np.ndarray): the error vector
            dt (float): time step

        Returns:
            np.ndarray: the control input
        """
        lambdaVec = self.ComputeBoundaryFunction()
        controlInput = 0
        for i in range(self.order_):
            if lambdaVec[i] < abs(errorVec[i]):
                print("\033[33m" + f"Warning: [FunnelControl] Out of the funnel. lambda{i}= {lambdaVec[i]}, error{i}= {errorVec[i]}" + "\033[0m")

            controlInput += self.param_.K[i] * lambdaVec[i] ** self.param_.c[i] * errorVec[i] / np.abs(lambdaVec[i] ** self.param_.c[i] - np.abs(errorVec[i]) ** self.param_.c[i])
        
        self.currentTime_ += dt

        return controlInput
    
    def ResetTime(self):
        """
        Reset the time for the boundary function
        """
        self.boundaryFunction_.Reset(self.currentTime_)
    
    # boundary functions ----------------------------------------------------
    class ExpBoundaryFunction(BoundaryFunction):
        """
        The class of exponential boundary function

        Formula:
            λi(t) = (λ0i - λ∞i) * exp(-αi * t) + λ∞i
        """
        def __init__(self, order: int, alpha: np.ndarray, lambda0: np.ndarray, lambdaInf: np.ndarray) -> None:
            super().__init__(FunnelControl.BoundaryFunction.Type.EXP, order)
            
            # parameters
            self.alpha_ = alpha
            self.lambda0_ = lambda0
            self.lambdaInf_ = lambdaInf

        def ComputeBoundsImpl(self, time: float) -> np.ndarray:
            """
            Compute the bounds

            Args:
                time (float): time

            Returns:
                np.ndarray: the bounds
            """
            lambdaVec = np.zeros(self.order_)
            for i in range(self.order_):
                lambdaVec[i] = (self.lambda0_[i] - self.lambdaInf_[i]) * np.exp(-self.alpha_[i] * time) + self.lambdaInf_[i]
            
            return lambdaVec
        
        def __str__(self) -> str:
            """
            Convert to string

            Returns:
                str: the string
            """
            return json.dumps({
                "ExpBoundaryFunction": {
                    "order": self.order_,
                    "alpha": self.alpha_.tolist(),
                    "lambda0": self.lambda0_.tolist(),
                    "lambdaInf": self.lambdaInf_.tolist()
                }})
        
    class CschBoundaryFunction(BoundaryFunction):
        """
        The class of hyperbolic cosecant boundary function
        It's used for Nobel Prescribed Performance Control(NPPC)

        Formula:
            λi(t) = csch(κi(t) * t + 1 / (λ0i - λ∞i)) + λ∞i
            κi(t) = κ∞i * (tanh(l0i * (t - t0i)) + 1) / 2
        """
        def __init__(self, order: int, kappaInf: np.ndarray, l0: np.ndarray, t0: np.ndarray, lambda0: np.ndarray, lambdaInf: np.ndarray) -> None:
            super().__init__(FunnelControl.BoundaryFunction.Type.CSCH, order)
            
            # parameters
            self.kappaInf_ = kappaInf
            self.l0_ = l0
            self.t0_ = t0
            self.lambda0_ = lambda0
            self.lambdaInf_ = lambdaInf

        def ComputeBoundsImpl(self, time: float) -> np.ndarray:
            """
            Compute the bounds

            Args:
                time (float): time

            Returns:
                np.ndarray: the bounds
            """
            lambdaVec = np.zeros(self.order_)
            for i in range(self.order_):
                kappa = self.kappaInf_[i] * (np.tanh(self.l0_[i] * (time - self.t0_[i])) + 1) / 2
                lambdaVec[i] = 1 / np.sinh(kappa * time + np.arcsinh(1 / (self.lambda0_[i] - self.lambdaInf_[i]))) + self.lambdaInf_[i]
                if(i == 0):
                    DebugDataLogger.PushData(kappa, "kappa")
            
            return lambdaVec
        
        def __str__(self) -> str:
            """
            Convert to string

            Returns:
                str: the string
            """
            return json.dumps({
                "CschBoundaryFunction": {
                    "order": self.order_,
                    "kappaInf": self.kappaInf_.tolist(),
                    "l0": self.l0_.tolist(),
                    "t0": self.t0_.tolist(),
                    "lambda0": self.lambda0_.tolist(),
                    "lambdaInf": self.lambdaInf_.tolist()
                }})
        
    class UserDefinedBoundaryFunction(BoundaryFunction):
        """
        The class of user-defined boundary function
        """
        def __init__(self, order: int, func, param) -> None:
            super().__init__(FunnelControl.BoundaryFunction.Type.USER_DEFINED, order)
            
            # parameters
            self.func_ = func
            self.param_ = param

        def ComputeBoundsImpl(self, time: float) -> np.ndarray:
            """
            Compute the bounds

            Args:
                time (float): time

            Returns:
                np.ndarray: the bounds
            """
            return self.func_(time, self.param_, self.order_)
        
        def __str__(self) -> str:
            """
            Convert to string

            Returns:
                str: the string
            """
            return json.dumps({
                "UserDefinedBoundaryFunction": {
                    "order": self.order_,
                    "func": self.func_,
                    "param": self.param_
                }})


# ----------------------------------------------------------------------------
# * @file FunnelControl.py
# * History
# * -------
# * - 2024/09/11 New created.(By hoshina)
