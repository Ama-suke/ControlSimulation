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
        if len(self.dataBuffer_) == 0:  
            return
        
        with open(fileName, mode='w') as f:
            labels = ""
            for key, data in self.dataBuffer_.items():
                labels += key + ","
            labels = labels[:-1]
            f.write(labels + "\n")
            
            for i in range(len(data)):
                dataStr = ""
                for key, data in self.dataBuffer_.items():
                    dataStr += str(data[i]) + ","
                dataStr = dataStr[:-1]
                f.write(dataStr + "\n")
        self.dataBuffer_.clear()

    def GetData():
        pass

# ----------------------------------------------------------------------------
# * @file DataLogger.py
# * History
# * -------
# * - 2024/07/04 New created.(By hoshina)
