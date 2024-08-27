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
import json

from Lib.Utils.StateSpace import StateSpace
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator
from Control.Pendulum.Controller.PidController import PidController
from Control.Pendulum.Plant.Pendulum import Pendulum
from Control.Pendulum.Plant.EquivalentSaturateModel import EquivalentSaturateModel

class ControllerType(Enum):
    PID = 0

class PlantType(Enum):
    ACTUAL = 0
    EQUIVALENT = 1

class Parameter:
    dt = 0.01           # time step
    stopTime = 20       # simulation time

    solverType = StateSpace.SolverType.EULER  # solver type
    initialStates = [0, 0]    # initial states

    controllerType = ControllerType.PID
    plantType = PlantType.EQUIVALENT

    plantParam = Pendulum.Param(
        J = 0.01,   # moment of inertia of the pendulum
        m = 0.1,    # mass of the pendulum
        l = 0.5,    # length of the pendulum
        D = 0.01,   # damping coefficient
        g = 9.81,   # gravitational acceleration
        tauMin = -50,   # minimum input. Set "None" if no limit
        tauMax = 50     # maximum input. Set "None" if no limit
    )
    equivalentPlantParam = EquivalentSaturateModel.Param(
        m = plantParam.m,    # mass
        J = plantParam.J,   # moment of inertia
        D = plantParam.D,   # damping coefficient
        l = plantParam.l,    # length
        g = plantParam.g,   # gravitational acceleration
        tauMin = plantParam.tauMin,   # minimum input. Set "None" if no limit
        tauMax = plantParam.tauMax,    # maximum input. Set "None" if no limit
        Kp = 5.0,      # proportional gain
        Kd = 0.1,      # derivative gain
        wMin = -100,  # minimum quasi-state
        wMax = 100,   # maximum quasi-state
        deltaDDisabled = True  # True if the uncertainty ΔD is disabled
    )
    
    pidControllerParam = PidController.Param(
        Kp = 5.0,      # proportional gain for angle
        Ki = 0.0,      # integral gain for angle
        Kd = 0.01,      # derivative gain for angle
        plantParam=plantParam
    )

    referenceGeneratorParams = [
        StepGenerator.Param(
            stepValue=0,  # step value
            initialValue=0,  # initial value
            startTimeStep=stopTime / dt / 2  # time step of the step
        )
    ]

    disturbanceGeneratorParams = [
        ImpulseGenerator.Param(
            amplitude=50.0,  # amplitude of the impulse
            startTimeStep=stopTime / dt * 1/4  # time step of the impulse
        )
    ]

    @staticmethod
    def SaveToFile(filename: str):
        """
        Save the parameters to a file

        Args:
            filename (str): file name
        """
        jsonStr = json.dumps({
            "Parameter": {
                "dt": Parameter.dt,
                "stopTime": Parameter.stopTime,
                "solverType": Parameter.solverType.name,
                "initialStates": Parameter.initialStates,
                "controllerType": Parameter.controllerType.name,
                "plantType": Parameter.plantType.name,
                "plantParam": json.loads(str(Parameter.plantParam)),
                "equivalentPlantParam": json.loads(str(Parameter.equivalentPlantParam)),
                "pidControllerParam": json.loads(str(Parameter.pidControllerParam)),
                "referenceGeneratorParams": [json.loads(str(p)) for p in Parameter.referenceGeneratorParams],
                "disturbanceGeneratorParams": [json.loads(str(p)) for p in Parameter.disturbanceGeneratorParams]
            }
        }, indent=4)

        with open(filename, "w") as f:
            f.write(jsonStr)

# ----------------------------------------------------------------------------
# * @file Parameter.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
