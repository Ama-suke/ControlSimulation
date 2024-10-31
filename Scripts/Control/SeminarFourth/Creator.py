#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Creator.py
# * @brief Creator class
# * @author hoshina
# * @date 2024/10/31
# * @details 
# *
# ----------------------------------------------------------------------------

from Control.Abstract.Plant import Plant
from Control.Abstract.Controller import Controller

from Control.SeminarFourth.Parameter import Parameter
from Control.SeminarFourth.Plant.MassSpringDamper import MassSpringDamper
from Control.SeminarFourth.Controller.StateObserver import StateObserver

def CreatePlant() -> Plant:
    """
    Create plant

    Returns:
        Plant: plant
    """
    return MassSpringDamper(Parameter.plant, solverType=Parameter.solverType,
                            initialState=Parameter.initialState)

def CreateController() -> Controller:
    """
    Create controller

    Returns:
        Controller: controller
    """
    return StateObserver(Parameter.controller)

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
# * - 2024/10/31 New created.(By hoshina)