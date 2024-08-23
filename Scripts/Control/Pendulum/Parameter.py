#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Parameter.py
# * @brief Parameters of pendulum control
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
from enum import Enum

from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator
from Control.Pendulum.Controller.PidController import PidController
from Control.Pendulum.Plant.Pendulum import Pendulum

class ControllerType(Enum):
    PID = 0

class Parameter:
    dt = 0.01           # time step
    stopTime = 20       # simulation time

    initialStates = [np.pi / 10, 0]    # initial states

    controllerType = ControllerType.PID

    plantParam = Pendulum.Param(
        J = 0.01,   # moment of inertia of the pendulum
        m = 0.1,    # mass of the pendulum
        l = 0.5,    # length of the pendulum
        D = 0.01,   # damping coefficient
        g = 9.81,   # gravitational acceleration
        tauMin = -0.5,   # minimum input. Set "None" if no limit
        tauMax = 0.5     # maximum input. Set "None" if no limit
    )
    pidControllerParam = PidController.Param(
        Kp = 5.0,      # proportional gain for angle
        Ki = 0.0,      # integral gain for angle
        Kd = 0.1      # derivative gain for angle
    )

    referenceGeneratorParams = [
        StepGenerator.Param(
            stepValue=0,  # step value
            initialValue=0,  # initial value
            startTimeStep=0  # time step of the step
        )
    ]

    disturbanceGeneratorParams = [
        ImpulseGenerator.Param(
            amplitude=0.1,  # amplitude of the impulse
            startTimeStep=stopTime / dt * 1/4  # time step of the impulse
        )
    ]


# ----------------------------------------------------------------------------
# * @file Parameter.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
