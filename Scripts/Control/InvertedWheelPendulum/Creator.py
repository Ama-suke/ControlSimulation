#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file Creator.py
# * @brief Object Creator for inverted wheel pendulum
# * @author hoshina
# * @date 2024/08/22
# * @details 
# *
# ----------------------------------------------------------------------------

from Lib.SignalGenerator.SignalGenerator import SignalGenerator
from Control.Abstract.Plant import Plant
from Control.Abstract.Controller import Controller

from Control.InvertedWheelPendulum.Parameter import Parameter, ControllerType
from Control.InvertedWheelPendulum.Plant.InvertedWheelPendulum import InvertedWheelPendulum
from Control.InvertedWheelPendulum.Controller.PidController import PidController
from Control.InvertedWheelPendulum.Controller.OperatorController import OperatorController
from Control.InvertedWheelPendulum.Controller.MorohoshiController import MorohoshiController
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator

def CreatePlant() -> Plant:
    """
    Create inverted wheel pendulum plant
    """
    return InvertedWheelPendulum(Parameter().plantParam, initialState=Parameter().initialStates)

def CreateController() -> Controller:
    """
    Create controller
    """
    p = Parameter()
    if p.controllerType == ControllerType.PID:
        return PidController(p.pidControllerParam)
    elif p.controllerType == ControllerType.OPERATOR:
        return OperatorController(p.operatorParam)
    elif p.controllerType == ControllerType.MOROHOSHI:
        return MorohoshiController(p.morohoshiParam)
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
# * - 2024/08/22 New created.(By hoshina)
