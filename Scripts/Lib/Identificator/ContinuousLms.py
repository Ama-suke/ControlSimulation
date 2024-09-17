#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file ContinuousLms.py
# * @brief LMS algorithm for continuous system identification
# * @author hoshina
# * @date 2024/08/04
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from pendulumStabilize.scripts.Identificator.Identificator import Identificator
from Utils.StateSpace import StateSpace
from Identificator.Utils.LeastMeanSquare import LeastMeanSquare
import math
from Utils.DataLogger import DataLogger

class ContinuousLms(Identificator):
    """
    LMS algorithm for continuous system identification
    
    Constructor:
        ContinuousLms(param)
    """

    class Param(Identificator.Param):
        def __init__(self, systemOrder: int, sampleNum: int, initialP: np.ndarray, isOnline: bool, filterLambda: float, dt: float) -> None:
            """
            constructor

            Args:
                systemOrder (int): order of system
                sampleNum (int): number of samples
                initialP (np.ndarray): initial P matrix
                isOnline (bool): online or offline
                filterLambda (float): filter parameter (filterLambda > 0)
                dt (float): time step
            """
            self.outputOrder = 1
            self.lmsParam = LeastMeanSquare.Param(
                paramNum=systemOrder * 2, # alpha and beta 
                sampleNum=sampleNum, 
                initialP=initialP, 
                isOnline=isOnline
            )
            self.systemOrder = systemOrder
            self.filterLambda = filterLambda
            self.dt = dt

            # create filter coefficient vector
            p = -(1 - filterLambda * dt / 2) / (1 + filterLambda * dt / 2)
            self.filterCoef = np.zeros(systemOrder)
            for i in range(systemOrder):
                self.filterCoef[i] = math.comb(systemOrder, i) * p ** (systemOrder - i)
            self.filterGain = 1 / (1 + p) ** systemOrder

    def __init__(self, param: Param):
        super().__init__(param)
        self.param_ = param
        self.lms_ = LeastMeanSquare(param.lmsParam)

        self.modelAlpha_ = np.zeros(param.systemOrder)
        self.modelBeta_ = np.zeros(param.systemOrder)
        self.model_ = StateSpace(self.modelStateEquation)

        self.ResetOutput()

    # private ---------------------------------------------------------------
    def IdentifySystemModelImpl(self, curInputs: np.ndarray, curOutputs: np.ndarray, dt: float):
        # coordinate variables
        output = curOutputs[0] # only supported 1st order
        self.inputFilterState_ = self.ComputeNextFilterState(self.inputFilterState_, curInputs, self.param_)
        self.outputFilterState_ = self.ComputeNextFilterState(self.outputFilterState_, output, self.param_)
        coordinatedInput = self.ComputeCoordinatedState(self.inputFilterState_, curInputs, self.param_)
        coordinatedOutput = self.ComputeCoordinatedState(self.outputFilterState_, output, self.param_)
        for i in range(len(coordinatedInput)):
            DataLogger.PushData(coordinatedInput[i], "coordinatedInput" + str(i))
        for i in range(len(coordinatedOutput)):
            DataLogger.PushData(coordinatedOutput[i], "coordinatedOutput" + str(i))

        # identify system model
        identifyInputs = np.concatenate((-coordinatedOutput[:-1], coordinatedInput[:-1]))
        identifyOutputs = coordinatedOutput[-1]
        self.lms_.DoIdentification(identifyInputs, identifyOutputs)

        theta = self.lms_.GetIdentifiedParam()
        self.modelAlpha_ = theta[:self.param_.systemOrder]
        self.modelBeta_ = theta[self.param_.systemOrder:]

        for i in range(len(theta)):
            DataLogger.PushData(theta[i], "theta" + str(i))

    def UpdateEstimatedOutputImpl(self, curInputs: np.ndarray, dt: float):
        self.modelState_ = self.model_.ComputeNextState(self.modelState_, curInputs, dt, self.param_)
        self.estimatedOutput_[0] = np.dot(self.modelBeta_, self.modelState_)

    def ResetOutputImpl(self):
        self.outputFilterState_ = np.zeros(self.param_.systemOrder)
        self.inputFilterState_ = np.zeros(self.param_.systemOrder)
        self.modelState_ = np.zeros(self.param_.systemOrder)

    @staticmethod
    def ComputeNextFilterState(curState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        p = param

        nextFilterState = np.zeros(p.systemOrder)
        for i in range(p.systemOrder - 1):
            nextFilterState[i] = curState[i + 1]
        nextFilterState[p.systemOrder - 1] = p.filterGain * curInput - np.dot(p.filterCoef, curState)

        return nextFilterState
        
    @staticmethod
    def ComputeCoordinatedState(curFilterState: np.ndarray, curInput: np.ndarray, param: Param) -> np.ndarray:
        p = param
        n = p.systemOrder

        newestState = ContinuousLms.ComputeNextFilterState(curFilterState, curInput, p)[-1]
        
        curState = np.zeros(n + 1)
        for i in range(n):
            curState[i] = curFilterState[i]
        curState[n] = newestState

        coordinatedState = np.zeros(n + 1)
        for i in range(p.systemOrder + 1):
            gain = (p.dt / 2) ** (n - i)
            # (z + 1)^(n - i) * (z - 1)^i * x(z)
            for k in range(n + 1):
                coef = 0
                for j in range(max(0, n - k - i), min(n - k, n - i) + 1):
                    tmpCoef = (math.comb(n - i, j) * math.comb(i, n - k - j) * ((-1) ** (n - k - j)))
                    coef += tmpCoef
                coordinatedState[i] += gain * coef * curState[k]

        return coordinatedState
                

    def modelStateEquation(self, curState: np.ndarray, curInput: np.ndarray, param: any) -> np.ndarray:
        p = param

        diffState = np.zeros(p.systemOrder)
        for i in range(p.systemOrder - 1):
            diffState[i] = curState[i + 1]
        diffState[p.systemOrder - 1] = -np.dot(self.modelAlpha_, curState) + curInput        

        return diffState

# ----------------------------------------------------------------------------
# * @file ContinuousLms.py
# * History
# * -------
# * - 2024/08/04 New created.(By hoshina)
