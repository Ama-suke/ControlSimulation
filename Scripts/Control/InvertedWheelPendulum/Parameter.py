#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Parameter.py
# * @brief define parameters
# * @author hoshina
# * @date 2024/07/01
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from enum import Enum

from Control.InvertedWheelPendulum.Plant.InvertedWheelPendulum import InvertedWheelPendulum
from Control.InvertedWheelPendulum.Controller.PidController import PidController
from Control.InvertedWheelPendulum.Controller.OperatorController import OperatorController
from Control.InvertedWheelPendulum.Controller.MorohoshiController import MorohoshiController
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator

class ControllerType(Enum):
    PID = 0
    OPERATOR = 1
    MOROHOSHI = 2

class Parameter():
    dt = 0.01           # time step
    stopTime = 20       # simulation time

    initialStates = [0, 0, 0, 0]    # initial states

    controllerType = ControllerType.MOROHOSHI

    plantParam = InvertedWheelPendulum.Param(
        m_w = 0.01,     # mass of the wheel
        J_w = 0.001,    # moment of inertia of the wheel
        D_w = 0.01,     # damping coefficient of the wheel
        m_p = 0.3,      # mass of the pendulum
        J_p = 0.01,     # moment of inertia of the pendulum
        D_p = 0.01,     # damping coefficient of the pendulum
        r = 0.03,       # radius of the wheel
        l_g = 0.06,     # distance from the wheel to the center of gravity of the pendulum
        g = 9.81,       # gravitational acceleration
        tauMin = -100,   # minimum input. Set "None" if no limit
        tauMax = 100     # maximum input. Set "None" if no limit
    )
    pidControllerParam = PidController.Param(
        KpAngle = 5.0,      # proportional gain for angle
        KiAngle = 0.0,      # integral gain for angle
        KdAngle = 0.5,      # derivative gain for angle
        KpPos = -5.0,       # proportional gain for displacement
        KiPos = -0.00,      # integral gain for displacement
        KdPos = -2.5        # derivative gain for displacement
    )
    operatorParam = OperatorController.Param(
        plantParam = plantParam,
        KpAngle = 5.0,     # proportional gain for angle
        KdAngle = 0.5,     # derivative gain for angle
        KpPos = 5.0,       # proportional gain for position
        KdPos = 0.06,      # derivative gain for position
        kappa = 5.0,       # gain for position feedback
        Lx1 = 200.0,       # observer gain for 1st state
        Lx2 = 200.0,       # observer gain for 2nd state
        Lx3 = 100.0,       # observer gain for 3rd state
        Lx4 = 100.0        # observer gain for 4th state
    )
    morohoshiParam = MorohoshiController.Param(
        plantParam = plantParam,
        k1=100,     # gain for angle
        k2=1,     # gain for angular velocity
        k3=1,     # gain for position
        lpfTau=0.01,   # time constant of low-pass filter
    )
    referenceGenerators = [
        StepGenerator.Param(
            stepValue = 0.0,      # step value
            initialValue = 0.0,   # initial value
            startTimeStep = stopTime / (2 * dt)   # step time
        )
    ]
    disturbanceGenerators = [
        ImpulseGenerator.Param(
            amplitude = 0.5,    # amplitude
            startTimeStep = stopTime / (dt) * 0   # start time step
        )
    ]

    @staticmethod
    def SaveToFile(filename: str):
        """
        Save the parameters to a file

        Args:
            filename (str): file name
        """
        pass

# ----------------------------------------------------------------------------
# * @file Parameter.py
# * History
# * -------
# * - 2024/07/01 New created.(By hoshina)
