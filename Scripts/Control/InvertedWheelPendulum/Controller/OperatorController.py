#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * @brief rcf controller based on operator theory
# * @author hoshina
# * @date 2024/07/04
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from numpy.core.multiarray import array as array
from Control.Abstract.Controller import Controller
from Control.InvertedWheelPendulum.Plant.InvertedWheelPendulum import InvertedWheelPendulum
from Lib.Utils.StateSpace import StateSpace
from Lib.Utils.DataLogger import DataLogger

class OperatorController(Controller):
    pass

class OperatorController(Controller):
    """
    Controller class of operator controller

    Constructor:
        OperatorController(param)

    Methods:
        ComputeControlInput: Compute control input
            controlInput = ComputeControlInput(refState, curState, prevSatInput, dt)
    """

    class Param(Controller.Param):
        """
        Parameters of operator controller
        """
        def __init__(self, plantParam: InvertedWheelPendulum.Param, KpAngle: float, KdAngle: float, KpPos: float, KdPos: float, kappa: float, Lx1: float, Lx2: float, Lx3: float, Lx4: float) -> None:
            """
            constructor

            Args:
                plantParam (InvertedWheelPendulum.Param): plant parameters
                KpAngle (float): proportional gain for angle
                KdAngle (float): derivative gain for angle
                KpPos (float): proportional gain for position
                KdPos (float): derivative gain for position
                kappa (float): gain for position feedback
            """
            self.plantParam = plantParam
            # feedback gains
            self.KpAngle = KpAngle
            self.KdAngle = KdAngle
            self.KpPos = KpPos
            self.KdPos = KdPos
            self.kappa = kappa
            # observer gains
            self.Lx1 = Lx1
            self.Lx2 = Lx2
            self.Lx3 = Lx3
            self.Lx4 = Lx4

    class OperatorR():
        """
        Operator R
        """

        def __init__(self, param: OperatorController.Param) -> None:
            """
            constructor

            Args:
                param (Param): parameters
            """
            self.param_ = param
            self.operatorFR_ = StateSpace(self.operatorFRStateEquation, StateSpace.SolverType.RUNGE_KUTTA)
            self.xRForward = np.zeros(5)
            self.xRInverse = np.zeros(5)

        def ComputeForward(self, u: np.ndarray, dt: float) -> np.ndarray:
            """
            Compute forward operator

            Args:
                u (np.ndarray): input
                dt (float): time step

            Returns:
                np.ndarray: output
            """
            self.xRForward = self.operatorFR_.ComputeNextState(self.xRForward, u, dt, self.param_)
            e = self.ComputeHR(self.xRForward) + self.ComputeGR(u)
            
            return e
        
        def ComputeInverse(self, e: np.ndarray, prevSatInput: np.ndarray, dt: float) -> np.ndarray:
            """
            Compute inverse operator

            Args:
                e (np.ndarray): error
                dt (float): time step

            Returns:
                np.ndarray: input
            """

            self.xRInverse = self.operatorFR_.ComputeNextState(self.xRInverse, prevSatInput, dt, self.param_)
            u = self.ComputeGRInv(-self.ComputeHR(self.xRInverse) + e)

            return u
        
        def operatorFRStateEquation(self, curState: np.ndarray, curInput: np.ndarray, param: OperatorController.Param) -> np.ndarray:
            """
            Compute forward and reverse operator

            Args:
                curState (np.ndarray): current state
                curInput (np.ndarray): current input
                param (Param): parameters

            Returns:
                np.ndarray: output
            """
            input = np.array([0, 0, curInput[0]])
            return OperatorController.observerStateEquation(curState, input, param)
        
        def ComputeHR(self, xR: np.ndarray) -> np.ndarray:
            """
            Compute forward and reverse operator

            Args:
                e (np.ndarray): error

            Returns:
                np.ndarray: output
            """
            p = self.param_

            e = 0
            return e
        
        def ComputeGR(self, u: np.ndarray) -> np.ndarray:
            """
            Compute forward and reverse operator

            Args:
                e (np.ndarray): error

            Returns:
                np.ndarray: output
            """
            p = self.param_

            e = -1 / p.KpPos * u
            return e
        
        def ComputeGRInv(self, e: np.ndarray) -> np.ndarray:
            """
            Compute forward and reverse operator

            Args:
                e (np.ndarray): error

            Returns:
                np.ndarray: output
            """
            p = self.param_

            u = -p.KpPos * e
            return u

    class OperatorS():
        """
        Operator S
        """

        def __init__(self, param: OperatorController.Param) -> None:
            """
            constructor

            Args:
                param (Param): parameters
            """
            self.param_ = param
            self.operatorFS_ = StateSpace(self.operatorFSStateEquation, StateSpace.SolverType.RUNGE_KUTTA)
            self.xSForward = np.zeros(5)

        def ComputeForward(self, y: np.ndarray, dt: float) -> np.ndarray:
            """
            Compute forward operator

            Args:
                u (np.ndarray): input
                dt (float): time step

            Returns:
                np.ndarray: output
            """

            self.xSForward = self.operatorFS_.ComputeNextState(self.xSForward, y, dt, self.param_)
            b = self.ComputeHS(self.xSForward, y)
            
            return b
        
        def operatorFSStateEquation(self, curState: np.ndarray, curInput: np.ndarray, param: OperatorController.Param) -> np.ndarray:
            """
            Compute forward and reverse operator

            Args:
                curState (np.ndarray): current state
                curInput (np.ndarray): current input
                param (Param): parameters

            Returns:
                np.ndarray: output
            """
            input = np.array([curInput[0], curInput[1], 0])
            return OperatorController.observerStateEquation(curState, input, param)
        
        def ComputeHS(self, xS: np.ndarray, y: np.ndarray) -> np.ndarray:
            """
            Compute forward and reverse operator

            Args:
                e (np.ndarray): error
            """
            p = self.param_
            pp = p.plantParam

            theta = y[0]
            x = y[1]

            cos_t = np.cos(theta)
            sin_t = np.sin(theta)

            z1 = pp.J_p + pp.m_p * pp.l_g ** 2 + pp.J_w + (pp.m_p + pp.m_w) * pp.r ** 2
            z2 = pp.m_p * pp.l_g * pp.r
            z3 = pp.J_w + (pp.m_p + pp.m_w) * pp.r ** 2

            b = np.exp(-p.kappa * np.abs(theta) ** 2) * x \
                + z3 / (z2 * cos_t + z3) / p.KpPos * pp.m_p * pp.g * pp.l_g * sin_t \
                + p.KpAngle / p.KpPos * theta \
                + p.KdAngle / p.KpPos * xS[2] \
                + p.KdPos / p.KpPos * xS[3] * np.exp(-p.kappa * np.abs(theta) ** 2)
            
            return b

    def __init__(self, controllerParam: Param) -> None:
        """
        constructor

        Args:
            controllerParam (Param): controller parameters
        """
        super().__init__(controllerParam)
        self.operatorR_ = self.OperatorR(controllerParam)
        self.operatorS_ = self.OperatorS(controllerParam)
        self.prevTheta_ = 0.0
        self.prevPhi_ = 0.0
        self.controlInput_ = 0.0

    def ComputeControlInputImpl(self, refState: np.ndarray, curState: np.ndarray, prevSatInput: np.ndarray, dt: float, param: Param) -> np.ndarray:
        """
        Compute control input

        Args:
            refState (np.ndarray): reference state
            curState (np.ndarray): current state
            prevSatInput (np.ndarray): previous saturated input
            dt (float): time step
            param (Param): controller parameters

        Returns:
            np.ndarray: control input
        """
        b = self.operatorS_.ComputeForward(curState, dt)
        e = refState - b
        self.controlInput_ = self.operatorR_.ComputeInverse(e, prevSatInput, dt)

        return self.controlInput_
    
    def PushStateToLoggerImpl(self, refState: np.array, dataLogger: DataLogger) -> None:
        dataLogger.PushData(refState[0], "refX")
    
    @staticmethod
    def observerStateEquation(curState: np.ndarray, curInput: np.ndarray, param) -> np.ndarray:
        """
        Compute forward and reverse operator

        Args:
            curState (np.ndarray): current state
            curInput (np.ndarray): current input
            param (Param): parameters

        Returns:
            np.ndarray: output
        """
        p = param
        pp = p.plantParam

        thetaHat = curState[0]
        phiHat = curState[1]
        thetaDotHat = curState[2]
        phiDotHat = curState[3]

        sensTheta = curInput[0]
        sensPhi = (curInput[1] - curInput[0]) / pp.r
        motorTorque = curInput[2]

        cos_th = np.cos(thetaHat)
        sin_th = np.sin(thetaHat)

        z1 = pp.J_p + pp.m_p * pp.l_g ** 2 + pp.J_w + (pp.m_p + pp.m_w) * pp.r ** 2
        z2 = pp.m_p * pp.l_g * pp.r
        z3 = pp.J_w + (pp.m_p + pp.m_w) * pp.r ** 2

        inertiaMatrix = np.array([
            [z1 + 2 * z2, 
             z3 + z2],
            [z3 + z2, 
             z3]
        ])
        momentVector = np.array([
            pp.m_p * pp.g * pp.l_g * np.sin(sensTheta),
            motorTorque
        ])

        diffState = np.zeros(5)

        diffState[0] = thetaDotHat + p.Lx1 * (sensTheta - thetaHat)
        diffState[1] = phiDotHat + p.Lx2 * (sensPhi - phiHat)
        diffState[2:4] = np.linalg.solve(inertiaMatrix, momentVector)
        diffState[2] += p.Lx3 * (sensTheta - thetaHat)
        diffState[3] += p.Lx4 * (sensPhi - phiHat)

        return diffState

# ----------------------------------------------------------------------------
# * @file OperatorController.py
# * History
# * -------
# * - 2024/07/04 New created.(By hoshina)
