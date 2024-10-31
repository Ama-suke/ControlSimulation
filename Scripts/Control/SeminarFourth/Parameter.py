#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Parameter.py
# * @brief Parameter class
# * @author hoshina
# * @date 2024/10/31
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
from Control.SeminarFourth.Plant.MassSpringDamper import MassSpringDamper
from Control.SeminarFourth.Controller.StateObserver import StateObserver

class Parameter:
    dt = 0.001
    stopTime = 10.0

    solverType = StateSpace.SolverType.RUNGE_KUTTA
    initialState = np.array([0.1, 0.0])

    plant = MassSpringDamper.Param(
        mass=1.0,
        springCoef=1.0,
        viscousCoef=0.1
    )

    # codimental observer
    # controller = StateObserver.Param(
    #     plantParam=plant,
    #     observerGains=np.array([100, 100]),
    #     type=StateObserver.Type.CODIMENTAL
    # )
    # minimum order observer
    # controller = StateObserver.Param(
    #     plantParam=plant,
    #     observerGains=np.array([-10]),
    #     type=StateObserver.Type.MINIMUM_ORDER
    # )
    # modern disturbance observer
    # controller = StateObserver.Param(
    #     plantParam=plant,
    #     observerGains=np.array([50, 500, -500]),
    #     type=StateObserver.Type.MODERN_DISTURBANCE
    # )
    # classical disturbance observer
    controller = StateObserver.Param(
        plantParam=plant,
        observerGains=np.array([20, 50]),
        type=StateObserver.Type.CLASSICAL_DISTURBANCE
    )

    referenceGenerators = [
        SweepSinGenerator.Param(
            amplitude=1.0,
            startFreq=0.01,
            stopFreq=1.0,
            finishTime=stopTime
        )
    ]

    disturbanceGenerators = [
        StepGenerator.Param(
            stepValue=1,
            initialValue=0.0,
            startTimeStep=int(stopTime / dt / 2)
        ),
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
# * - 2024/10/31 New created.(By hoshina)
