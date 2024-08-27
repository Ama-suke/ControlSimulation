#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file DebugDataLogger.py
# * @brief Global data logger. for debug
# * @author hoshina
# * @date 2024/08/23
# * @details 
# *
# ----------------------------------------------------------------------------

from Lib.Utils.DataLogger import DataLogger

class DebugDataLogger:
    """
    Global data logger
    """
    
    logger_ = DataLogger()

    @staticmethod
    def PushData(data: float, key: str) -> None:
        """
        Push data to the global logger

        Args:
            data (np.ndarray): data to be logged
            key (str): key to identify the data
        """
        DebugDataLogger.logger_.PushData(data, key)

    @staticmethod
    def SaveLoggedData(fileName: str) -> None:
        """
        Save logged data to the file

        Args:
            fileName (str): file name
        """
        DebugDataLogger.logger_.SaveLoggedData(fileName)

# ----------------------------------------------------------------------------
# * @file DebugDataLogger.py
# * History
# * -------
# * - 2024/08/23 New created.(By hoshina)
