#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file DataLogger.py
# * @brief Class to log temporary data as csv file
# * @author hoshina
# * @date 2024/07/04
# * @details 
# *
# ----------------------------------------------------------------------------

import numpy as np

class DataLogger:
    """
    Class to log data as csv file

    Constructor:
        DataLogger()

    Methods:
        LogData: Log data
            LogData(data, fileName)
    """

    def __init__(self) -> None:
        """
        constructor
        """
        self.dataBuffer_ = {}
    
    def PushData(self, data: float, key: str) -> None:
        """
        Push data to the buffer

        Args:
            data (np.ndarray): data to be logged
            key (str): key to identify the data
        """
        if self.dataBuffer_.get(key) is None:
            self.dataBuffer_[key] = [data]
        else:
            self.dataBuffer_[key].append(data)

    def SaveLoggedData(self, fileName: str) -> None:
        """
        Save logged data to the file

        Args:
            fileName (str): file name
        """
        csvStr = ""
        for key, data in self.dataBuffer_.items():
            csvStr += key + ","
        csvStr = csvStr[:-1]
        csvStr += "\n"
        for i in range(len(data)):
            for key, data in self.dataBuffer_.items():
                if i >= len(data):
                    break
                
                csvStr += str(data[i]) + ","
            csvStr = csvStr[:-1]
            csvStr += "\n"

        with open(fileName, mode='w') as f:
            f.write(csvStr)

    def GetData():
        pass

# ----------------------------------------------------------------------------
# * @file DataLogger.py
# * History
# * -------
# * - 2024/07/04 New created.(By hoshina)
