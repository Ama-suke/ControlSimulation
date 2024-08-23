#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file stateSpace.py
# * @brief State space model for modern control eng.
# * @author hoshina
# * @date 2024/07/02
# * @details 
# *
# ----------------------------------------------------------------------------

from enum import Enum
import numpy as np

class StateSpace():
    """
    Modern control eng. state space model

    Constructor:
        StateSpace(fStateEquation, solverType = SolverType.RUNGE_KUTTA)

    Methods:
        StateUpdate: Return next state of dynamical system
            nextState = StateUpdate(curState, curInput, dt, param)
    """

    class SolverType(Enum):
        EULER = 0           # Euler method. 1st order
        RUNGE_KUTTA = 1     # Runge-Kutta method. 4th order

    def __init__(self, fStateEquation, solverType: SolverType = SolverType.RUNGE_KUTTA) -> None:
        """
        constructor

        Args:
            fStateEquation (function): state equation
            solverType (SolverType, optional): solver type. Defaults to SolverType.RUNGE_KUTTA.
        """

        if solverType == StateSpace.SolverType.EULER:
            self.fSolver_ = self.SolveEuler
        elif solverType == StateSpace.SolverType.RUNGE_KUTTA:
            self.fSolver_ = self.SolveRungeKutta

        self.fStateEquation_ = fStateEquation

    def ComputeNextState(self, curState, curInput, dt, param):
        """
        State update

        Args:
            curState (np.array): current state
            curInput (np.array): current input
            dt (float): time step
            param (np.array): parameters

        Returns:
            np.array: next state
        """

        nextState = self.fSolver_(curState, curInput, dt, self.fStateEquation_, param)
        return nextState

    # private ------------------------------------------------------
    @staticmethod
    def SolveEuler(curState, curInput, dt, fStateEquation, param):
        nextState = curState + fStateEquation(curState, curInput, param) * dt
        return nextState
    
    @staticmethod
    def SolveRungeKutta(curState, curInput, dt, fStateEquation, param):
        k1 = fStateEquation(curState, curInput, param)
        k2 = fStateEquation(curState + k1 * dt / 2, curInput, param)
        k3 = fStateEquation(curState + k2 * dt / 2, curInput, param)
        k4 = fStateEquation(curState + k3 * dt, curInput, param)
        nextState = curState + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
        return nextState

# ----------------------------------------------------------------------------
# * @file stateSpace.py
# * History
# * -------
# * - 2024/07/02 New created.(By hoshina)