#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file LeastMeanSquare.py
# * @brief LMS algorithm system identification
# * @author hoshina
# * @date 2024/08/04
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np

class LeastMeanSquare:
    """
    LMS algorithm system identification

    Constructor:
        LeastMeanSquare(param)

    Methods:
        DoIdentification: Identify the system
            DoIdentification(curInputs, curOutputs)
        GetIdentifiedParam: Get the identified parameters
            GetIdentifiedParam()
    """

    class Param:
        def __init__(self, paramNum: int, sampleNum: int, initialP: np.ndarray, isOnline: bool) -> None:
            self.paramNum = paramNum    # number of parameters to identify
            self.sampleNum = sampleNum  # number of samples
            self.initialP = initialP    # initial P matrix
            self.isOnline = isOnline    # online or offline

    def __init__(self, param: Param):
        self.param_ = param

        self.P_ = param.initialP
        self.outputVector_ = np.zeros(param.sampleNum)
        self.inputMatrix_ = np.zeros((param.sampleNum, param.paramNum))
        self.dataCount_ = 0
        
        self.isFinished_ = False
        self.identifiedParam_ = np.zeros(param.paramNum)

    def DoIdentification(self, curInputs: np.ndarray, curOutputs: np.ndarray):
        if self.param_.isOnline:
            self.DoOnlineIdentification(curInputs, curOutputs)
        else:
            self.DoOfflineIdentification(curInputs, curOutputs)

    def GetIdentifiedParam(self):
        return self.identifiedParam_

    # private ---------------------------------------------------------------
    def DoOnlineIdentification(self, curInputs: np.ndarray, curOutputs: np.ndarray):
        theta = self.identifiedParam_
        inputMatrix = curInputs[:, np.newaxis]

        r = 1 + curInputs.T @ self.P_ @ inputMatrix
        theta += self.P_ @ curInputs * (curOutputs - inputMatrix.T @ theta) / r
        self.P_ += -(self.P_ @ inputMatrix @ inputMatrix.T @ self.P_) / r

        self.identifiedParam_ = theta

    def DoOfflineIdentification(self, curInputs: np.ndarray, curOutputs: np.ndarray):
        if self.isFinished_:
            return

        if self.dataCount_ < self.param_.sampleNum:
            self.inputMatrix_[self.dataCount_] = curInputs.T
            self.outputVector_[self.dataCount_] = curOutputs
            self.dataCount_ += 1
            return

        # calculate the identified parameters
        self.identifiedParam_ = np.linalg.inv(self.inputMatrix_.T @ self.inputMatrix_) @ self.inputMatrix_.T @ self.outputVector_

        self.isFinished_ = True

# ----------------------------------------------------------------------------
# * @file LeastMeanSquare.py
# * History
# * -------
# * - 2024/08/04 New created.(By hoshina)
