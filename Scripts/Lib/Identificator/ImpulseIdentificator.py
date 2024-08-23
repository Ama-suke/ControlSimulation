#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file ImpulseIdentificator.py
# * @brief identificator based Impulse response
# * @author hoshina
# * @date 2024/08/03
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from numpy.core.multiarray import array as array
from pendulumStabilize.scripts.Identificator.Identificator import Identificator
from Utils.StateSpace import StateSpace
from Utils.DataLogger import DataLogger

class ImpulseIdentificator(Identificator):
    """
    identificator based Impulse response

    Constructor:
        ImpulseIdentificator(param)

    Methods:
        IdentifySystemModelImpl: Identify the system
            IdentifySystemModelImpl(curInputs, curOutputs)
    """

    class Param(Identificator.Param):
        """
        Parameters of Impulse response identificator
        """
        def __init__(self, systemOrder: int) -> None:
            """
            constructor

            Args:
                outputOrder (int): order of output
                inputOrder (int): order of input
            """

            # only supported 1st order
            self.outputOrder = 1 
            self.systemOrder = systemOrder

    def __init__(self, param: Param) -> None:
        """
        constructor

        Args:
            param (Param): parameters of Impulse response identificator
        """
        super().__init__(param)
        self.dataCount_ = 0
        self.isIdentified_ = False

        self.outputArray_ = np.zeros(2 * param.systemOrder + 1)
        self.impulseGain_ = 0

        self.modelLambda_ = np.zeros(param.systemOrder)
        self.modelC_ = np.zeros(param.systemOrder)
        self.modelState_ = np.zeros(param.systemOrder)
        self.model_ = StateSpace(self.modelStateEquation)

    def IdentifySystemModelImpl(self, curInputs: np.array, curOutputs: np.array, dt: float) -> None:
        """
        Identify the system

        Args:
            curInputs (np.array): input signals in the current step
            curOutputs (np.array): output signals in the current step
            dt (float): time step
        """
        p = self.param_

        if self.isIdentified_:
            return

        if curInputs[0] != 0.0:
            self.dataCount_ = 0
            self.impulseGain_ = curInputs[0] * dt

        if self.dataCount_ < 2 * p.systemOrder + 1:
            # データをためる（手順2）
            self.outputArray_[self.dataCount_] = curOutputs[0] / self.impulseGain_

            self.dataCount_ += 1
            return
        
        # 所定数のデータがたまったら、モデルを推定する
        # 出力行列を特異値分解する（改良版手順1）
        HMat = np.zeros((p.systemOrder + 1, p.systemOrder + 1))
        for i in range(p.systemOrder + 1):
            for j in range(p.systemOrder + 1):
                HMat[i][j] = self.outputArray_[i + j]

        U, S, V = np.linalg.svd(HMat)

        # Uを使って\bar{A}を求める（改良版手順2）
        Un1 = U[0:p.systemOrder, 0:p.systemOrder]
        Un2 = U[1:p.systemOrder + 1, 0:p.systemOrder]
        ABar = np.linalg.solve(Un1, Un2)
        
        # ABarの固有値(\mu)を求める（改良版手順3）
        mu, eigVec = np.linalg.eig(ABar)
        print("mu: ", mu)

        # \muから\lambdaを求める（手順5）
        self.modelLambda_ = np.log(mu) / dt
        print("lambda: ", self.modelLambda_)

        # \からCを求める（手順6）
        coefMat = np.zeros((p.systemOrder, p.systemOrder), dtype=np.complex128)
        outVec = np.zeros(p.systemOrder)
        for i in range(p.systemOrder):
            coefMat[i] = np.power(mu, i)
            outVec[i] = self.outputArray_[i]
        self.modelC_ = np.linalg.solve(coefMat, outVec)
        print("c:", self.modelC_)

        self.isIdentified_ = True

    def UpdateEstimatedOutputImpl(self, curInputs: np.array, dt: float) -> None:
        """
        Update the estimated output

        Args:
            curInputs (np.array): input signals in the current step
            dt (float): time step
        """
        if not self.isIdentified_:
            return
        
        for i in range(self.param_.systemOrder):
            DataLogger.PushData(self.modelState_[i], "state" + str(i))

        self.modelState_ = self.model_.ComputeNextState(self.modelState_, curInputs, dt, self.param_)

        self.estimatedOutput_[0] = 0.0
        for i in range(self.param_.systemOrder):
            self.estimatedOutput_[0] += self.modelState_[i]

    def ResetOutputImpl(self) -> None:
        """
        Reset the estimated output
        """
        self.modelState_ = np.zeros(self.param_.systemOrder)
     
    def modelStateEquation(self, curState: np.array, curInput: np.array, param: any) -> np.array:
        """
        Model state equation

        Args:
            curInputs (np.array): input signals in the current step

        Returns:
            np.array: estimated output
        """
        diffModelState = np.zeros(param.systemOrder)
        for i in range(param.systemOrder):
            if not np.isnan(self.modelLambda_[i]):
                diffModelState[i] = self.modelLambda_[i] * curState[i] + self.modelC_[i] * curInput[0]

        return diffModelState

# ----------------------------------------------------------------------------
# * @file ImpulseIdentificator.py
# * History
# * -------
# * - 2024/08/03 New created.(By hoshina)
