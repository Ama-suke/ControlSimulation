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
from Control.Seminar.Plant.MassSpringDamper import MassSpringDamper
from Control.Seminar.Plant.PolePlacementModel import PolePlacementModel
from Control.Seminar.Plant.PoleZeroCancellationModel import PoleZeroCancellationModel
from Control.Seminar.Controller.PidPositionController import PidPositionController

class Parameter:
    class ModelType:
        MASS_SPRING_DAMPER = 0
        POLE_PLACEMENT = 1
        POLE_ZERO_CANCELLATION = 2

    dt = 0.01
    stopTime = 10

    solverType = StateSpace.SolverType.RUNGE_KUTTA
    initialState = np.array([0.0, 0.0])

    modelType = ModelType.MASS_SPRING_DAMPER

    plant = MassSpringDamper.Param(
        mass=1, 
        viscousCoef=1, 
        springCoef=1
    )

    polePlacement = PolePlacementModel.Param(
        MassSpringDamperParam=plant,
        poles=np.array([-5.0, -5.0, -5.0])
    )

    poleZeroCancellation = PoleZeroCancellationModel.Param(
        omega=5
    )

    # designed pole placement
    # controller = PidPositionController.Param(
    #     pidParam=PidServo.Param(
    #         Kp=-plant.springCoef + plant.mass * (polePlacement.poles[0] * polePlacement.poles[1] + polePlacement.poles[1] * polePlacement.poles[2] + polePlacement.poles[2] * polePlacement.poles[0]),
    #         Ki=-plant.mass * polePlacement.poles[0] * polePlacement.poles[1] * polePlacement.poles[2],
    #         Kd=-plant.viscousCoef - plant.mass * (polePlacement.poles[0] + polePlacement.poles[1] + polePlacement.poles[2]),
    #         tau=0
    #     )
    # )

    # designed pole zero cancellation
    controller = PidPositionController.Param(
        pidParam=PidServo.Param(
            Kp=plant.viscousCoef * poleZeroCancellation.omega,
            Ki=plant.springCoef * poleZeroCancellation.omega,
            Kd=plant.mass * poleZeroCancellation.omega,
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
