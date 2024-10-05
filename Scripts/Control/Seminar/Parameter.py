#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Parameter.py
# * @brief Parameters of control seminar
# * @author hoshina
# * @date 2024/10/05
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
import json

from Lib.Compensator.StateSpace import StateSpace
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator
from Lib.SignalGenerator.SinGenerator import SinGenerator
from Lib.SignalGenerator.SweepSinGenerator import SweepSinGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.Compensator.PidServo import PidServo
from Control.Seminar.Plant.massSpringDamper import MassSpringDamper
from Control.Seminar.Controller.PidPositionController import PidPositionController

class Parameter:
    dt = 0.01
    stopTime = 10

    solverType = StateSpace.SolverType.RUNGE_KUTTA
    initialState = np.array([0.0, 0.0])

    plant = MassSpringDamper.Param(
        mass=1, 
        viscousCoef=1, 
        springCoef=1
    )

    controller = PidPositionController.Param(
        pidParam=PidServo.Param(
            Kp=10,
            Ki=2,
            Kd=0,
            tau=0
        )
    )

    referenceGenerators = [
        StepGenerator.Param(
            stepValue=1,
            initialValue=0,
            startTimeStep=0
        )
    ]

    disturbanceGenerators = [
        StepGenerator.Param(
            stepValue=1,
            initialValue=0,
            startTimeStep=int(stopTime / dt / 2)
        )
    ]

    @staticmethod
    def SaveToFile(fileName: str) -> None:
        """
        Save parameter to file

        Args:
            fileName (str): file name
        """
        jsonStr = json.dumps({
            "Parameter": {
                "dt": Parameter.dt,
                "stopTime": Parameter.stopTime,
                "solverType": Parameter.solverType.name,
                "initialState": Parameter.initialState.tolist(),
                "plant": json.loads(str(Parameter.plant)),
                "controller": json.loads(str(Parameter.controller)),
                "referenceGenerators": [json.loads(str(p)) for p in Parameter.referenceGenerators],
                "disturbanceGenerators": [json.loads(str(p)) for p in Parameter.disturbanceGenerators]
            }
        }, indent=4)
        with open(fileName, "w") as f:
            f.write(jsonStr)

# ----------------------------------------------------------------------------
# * @file Parameter.py
# * History
# * -------
# * - 2024/10/05 New created.(By hoshina)
