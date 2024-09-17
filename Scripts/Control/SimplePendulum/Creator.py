#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Creator.py
# * @brief Creator of pendulum control
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

from Lib.SignalGenerator.SignalGenerator import SignalGenerator
from Control.Abstract.Plant import Plant
from Control.Abstract.Controller import Controller

from Control.SimplePendulum.Parameter import Parameter, ControllerType, PlantType
from Control.SimplePendulum.Plant.Pendulum import Pendulum
from Control.SimplePendulum.Plant.EquivalentSaturateModel import EquivalentSaturateModel
from Control.SimplePendulum.Controller.FrictionCompensation.PidController import PidController
from Control.SimplePendulum.Controller.FrictionCompensation.OperatorController import OperatorController
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator

def CreatePlant() -> Plant:
    """
    Create inverted wheel pendulum plant
    """
    p = Parameter()
    if p.plantType == PlantType.ACTUAL:
        return Pendulum(p.plantParam, solverType=p.solverType, initialState=p.initialStates)
    elif p.plantType == PlantType.EQUIVALENT:
        return EquivalentSaturateModel(p.equivalentPlantParam, solverType=p.solverType, initialState=p.initialStates)
    else:
        raise ValueError("Invalid plant type")

def CreateController() -> Controller:
    """
    Create controller
    """
    p = Parameter()
    if p.controllerType == ControllerType.PID:
        return PidController(p.pidControllerParam)
    elif p.controllerType == ControllerType.OPERATOR:
        return OperatorController(p.operatorControllerParam, p.operatorControllerQType)
    else:
        raise ValueError("Invalid controller type")

def GetParameter() -> Parameter:
    """
    Create parameter
    """
    return Parameter()

# ----------------------------------------------------------------------------
# * @file Creator.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
