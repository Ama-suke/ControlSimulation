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

from Lib.Compensator.StateSpace import StateSpace
from Lib.Compensator.FunnelControl import FunnelControl
from Lib.Compensator.PidServo import PidServo
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator
from Lib.SignalGenerator.SinGenerator import SinGenerator
from Lib.SignalGenerator.SweepSinGenerator import SweepSinGenerator
from Control.SimplePendulum.Controller.FrictionCompensation.PidController import PidController
from Control.SimplePendulum.Controller.FrictionCompensation.OperatorController import OperatorController
from Control.SimplePendulum.Plant.Pendulum import Pendulum
from Control.SimplePendulum.Plant.EquivalentSaturateModel import EquivalentSaturateModel

class ControllerType(Enum):
    PID = 0
    OPERATOR = 1

class PlantType(Enum):
    ACTUAL = 0
    EQUIVALENT = 1

class Parameter:
    dt = 0.001           # time step
    stopTime = 10       # simulation time

    solverType = StateSpace.SolverType.RUNGE_KUTTA  # solver type
    initialStates = [0, 0]    # initial states

    plantType = PlantType.ACTUAL

    plantParam = Pendulum.Param(
        J = 0.001,   # moment of inertia of the pendulum
        m = 0.054347,    # mass of the pendulum
        l = 0.065,    # length of the pendulum
        B = 0.001,   # damping coefficient
        g = 9.80665,   # gravitational acceleration
        f0 = 0.01,  # friction coefficient
        vSigma = 0.5,   # stribeck velocity
        R = 41.7,    # reduction ratio
        tauMin = -0.5,   # minimum input. Set "None" if no limit
        tauMax = 0.5,     # maximum input. Set "None" if no limit
        alpha = 0.50  # shape parameter of the saturation function
    )
    equivalentPlantParam = EquivalentSaturateModel.Param(
        baseParam = plantParam,
        Kp = 5.0,      # proportional gain
        Kd = 0.1,      # derivative gain
        wMin = -0.02,  # minimum quasi-state
        wMax = 0.02,   # maximum quasi-state
        deltaDDisabled = False  # True if the uncertainty ΔD is disabled
    )
    
    controllerType = ControllerType.OPERATOR

    pidControllerParam = PidController.Param(
        pidParam=PidServo.Param(
            Kp = 5.0,      # proportional gain for angle
            Ki = 0.0,      # integral gain for angle
            Kd = 0.1,      # derivative gain for angle
            tau = 0.005     # time constant of the low-pass filter
        ),
        plantParam=plantParam
    )
    operatorControllerParam = OperatorController.Param(
        pidParam = pidControllerParam.pidParam,  # parameters of the PID controller
        tauD = 0.005,     # time constant of the low-pass filter
        pR = np.array([5000.0, 200.0]),  # reference model poles
        gF = 1.0,       # gain of friction compensation
        theta = 0.0001,    # shape parameter of the sign function
        gamma = 10,    # shape parameter of the sign function
        xDeltaBar = 0.001,  # parameter of the funnel control
        funnelParam = FunnelControl.Param(
            K = np.array([pidControllerParam.pidParam.Kp, pidControllerParam.pidParam.Kd]),  # parameter of the funnel control
            c = np.array([0.5, 0.5]),  # parameter of the funnel control
        ),  # parameters of the funnel control
        boundaryFunction = FunnelControl.ExpBoundaryFunction(
            order = 2,  # order of the funnel control
            alpha = np.array([2, 2]),  # parameter of the funnel control
            lambda0 = np.array([0.01, 50]),  # parameter of the funnel control
            lambdaInf = np.array([0.001, 2]),  # parameter of the funnel control
        ),  # boundary function
        plantParam = plantParam # parameters of the plant
    )
    operatorControllerQType = OperatorController.QOperator.PD  # type of Q operator

    referenceGeneratorParams = [
        SweepSinGenerator.Param(
            amplitude=0.1,  # amplitude of the signal
            startFreq=0.1,  # start frequency
            stopFreq=10,  # stop frequency
            finishTime=stopTime  # finish time
        )
    ]

    disturbanceGeneratorParams = [
        ImpulseGenerator.Param(
            amplitude=0.0,  # amplitude of the impulse
            startTimeStep=1  # time step of the impulse
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
                "operatorControllerParam": json.loads(str(Parameter.operatorControllerParam)),
                "operatorControllerQType": Parameter.operatorControllerQType.name,
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
