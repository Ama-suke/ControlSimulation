#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Creator.py
# * @brief Creator of pendulum control
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Plant import Plant
from Control.Abstract.Controller import Controller
from Control.SeminarFirst.Parameter import Parameter

from Control.SeminarFirst.Plant.MassSpringDamper import MassSpringDamper
from Control.SeminarFirst.Plant.PolePlacementModel import PolePlacementModel
from Control.SeminarFirst.Plant.PoleZeroCancellationModel import PoleZeroCancellationModel
from Control.SeminarFirst.Controller.PidPositionController import PidPositionController
from Control.SeminarFirst.Controller.StraightFeedForward import StraightFeedForward
from Control.SeminarFirst.Parameter import Parameter

def CreatePlant() -> Plant:
    """
    Create plant

    Returns:
        Plant: plant
    """
    p = Parameter()
    if p.modelType == Parameter.ModelType.MASS_SPRING_DAMPER:
        return MassSpringDamper(p.plant, solverType = p.solverType, initialState = p.initialState)
    elif p.modelType == Parameter.ModelType.POLE_PLACEMENT:
        return PolePlacementModel(p.polePlacement, solverType = p.solverType)
    elif p.modelType == Parameter.ModelType.POLE_ZERO_CANCELLATION:
        return PoleZeroCancellationModel(p.poleZeroCancellation, solverType = p.solverType)
    else:
        raise ValueError("Invalid model type")

def CreateController() -> Controller:
    """
    Create controller

    Returns:
        Controller: controller
    """
    p = Parameter()
    if p.modelType == Parameter.ModelType.MASS_SPRING_DAMPER:
        return PidPositionController(p.controller)
    elif p.modelType == Parameter.ModelType.POLE_PLACEMENT:
        return StraightFeedForward()
    elif p.modelType == Parameter.ModelType.POLE_ZERO_CANCELLATION:
        return StraightFeedForward()
    else:
        raise ValueError("Invalid model type")

def GetParameter() -> Parameter:
    """
    Get parameter

    Returns:
        Parameter: parameter
    """
    return Parameter()

# ----------------------------------------------------------------------------
# * @file Creator.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
