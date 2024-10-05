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
from Control.Seminar.Parameter import Parameter

from Control.Seminar.Plant.massSpringDamper import MassSpringDamper
from Control.Seminar.Controller.PidPositionController import PidPositionController
from Control.Seminar.Parameter import Parameter

def CreatePlant() -> Plant:
    """
    Create plant

    Returns:
        Plant: plant
    """
    p = Parameter()
    return MassSpringDamper(p.plant, solverType = p.solverType, initialState = p.initialState)

def CreateController() -> Controller:
    """
    Create controller

    Returns:
        Controller: controller
    """
    p = Parameter()
    return PidPositionController(p.controller)

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
