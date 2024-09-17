#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file main.py
# * @brief main function
# * @author hoshina
# * @date 2024/08/22
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np
import os
import sys
import importlib.util
import datetime

from Lib.Utils.DataLogger import DataLogger
from DebugDataLogger import DebugDataLogger
from Control.Abstract.Controller import Controller
from Control.Abstract.Plant import Plant

from Lib.SignalGenerator.SignalGenerator import SignalGenerator
from Lib.SignalGenerator.ImpulseGenerator import ImpulseGenerator
from Lib.SignalGenerator.MSequenceGenerator import MSequenceGenerator
from Lib.SignalGenerator.StepGenerator import StepGenerator
from Lib.SignalGenerator.SinGenerator import SinGenerator

defaultProgram = "InvertedWheelPendulum"

def main(program: str):
    # move current directory to the this file's directory
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Load the program
    Creator = ImportProgram(program)
    parameter = Creator.GetParameter()
    plant: Plant = Creator.CreatePlant()
    controller: Controller = Creator.CreateController()

    # Create signal generators
    refGenerators = []
    ref = np.zeros(len(parameter.referenceGeneratorParams))
    for refParam in parameter.referenceGeneratorParams:
        refGenerators.append(CreateSignalGenerator(refParam))
    distGenerators = []
    dist = np.zeros(len(parameter.disturbanceGeneratorParams))
    for distParam in parameter.disturbanceGeneratorParams:
        distGenerators.append(CreateSignalGenerator(distParam))

    # Data logger for plot
    dataLogger = DataLogger()

    t = np.arange(0, parameter.stopTime, parameter.dt)

    controlInput = np.array([0.0])
    satControlInput = plant.GetSaturatedInput(controlInput)

    print("Start simulation")
    print("Processing...")
    for i in range(len(t)):
        # Generate signals
        for j in range(len(refGenerators)):
            ref[j] = refGenerators[j].GenerateSignal(i, parameter.dt)
        for j in range(len(distGenerators)):
            dist[j] = distGenerators[j].GenerateSignal(i, parameter.dt)

        # controller
        sens = plant.GetOutput(satControlInput)
        controlInput = controller.ComputeControlInput(ref, sens, 
                                                      satControlInput, 
                                                      parameter.dt)

        # compute input saturation
        satControlInput = plant.GetSaturatedInput(controlInput)
        inputs = np.concatenate((satControlInput, dist))

        # Store the state for plotting
        dataLogger.PushData(t[i], "time")
        plant.PushStateToLogger(inputs, dataLogger)
        controller.PushStateToLogger(ref, sens, dataLogger)

        # Update the plant state
        plant.UpdateState(inputs, parameter.dt)
    print("Done")

    # save the result to a file
    print("Save the result...")
    timeNow = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    dataPath = f"../Data/{timeNow}"
    if not os.path.exists(dataPath):
        os.makedirs(dataPath)
    dataLogger.SaveLoggedData(f"{dataPath}/result.csv")
    DebugDataLogger.SaveLoggedData(f"{dataPath}/debug.csv")
    parameter.SaveToFile(f"{dataPath}/parameter.json")

    # Delete old data if there are more than 10 directories
    dataDir = "../Data"
    dirs = [os.path.join(dataDir, d) for d in os.listdir(dataDir) if os.path.isdir(os.path.join(dataDir, d))]
    if len(dirs) > 10:
        dirs.sort()
        num_dirs_to_delete = len(dirs) - 10
        for i in range(num_dirs_to_delete):
            dirPath = dirs[i]
            for root, subdirs, files in os.walk(dirPath, topdown=False):
                for name in files:
                    os.remove(os.path.join(root, name))
                for name in subdirs:
                    os.rmdir(os.path.join(root, name))
            os.rmdir(dirPath)

    print("Finish")

def ImportProgram(program: str):
    module_name = "Creator"
    file_path = os.path.join(os.getcwd(), "Control", program, "Creator.py")
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"No such file or directory: '{file_path}'")
    
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    
    return module

def CreateSignalGenerator(param: SignalGenerator.Param) -> SignalGenerator:
    """
    Create reference signal generator
    """
    if isinstance(param, StepGenerator.Param):
        return StepGenerator(param)
    elif isinstance(param, ImpulseGenerator.Param):
        return ImpulseGenerator(param)
    elif isinstance(param, MSequenceGenerator.Param):
        return MSequenceGenerator(param)
    elif isinstance(param, SinGenerator.Param):
        return SinGenerator(param)
    else:
        raise ValueError("Invalid reference generator type")

if __name__ == "__main__":
    program = defaultProgram
    if len(sys.argv) == 2:
        program = sys.argv[1]
    else:
        print("Usage: python main.py <path to the program>")
        print(f"Using default program: {defaultProgram}")
        
    main(program=program)

# ----------------------------------------------------------------------------
# * @file main.py
# * History
# * -------
# * - 2024/08/22 New created.(By hoshina)
