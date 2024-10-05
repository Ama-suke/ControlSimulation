#!/usr/bin/env python
# ----------------------------------------------------------------------------
# * @file GraphPlotter.py
# * @brief Class to plot graphs using matplotlib
# * @author hoshina
# * @date 2024/10/05
# * @details 
# *
# ----------------------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np

from Lib.Utils.DataLogger import DataLogger

class GraphPlotter:
    class SinglePlot:
        def __init__(self, title: str = "", xLabel: str = "", yLabel: str = "", 
                     figWidth: float = 6, hasGrid: bool = True) -> None:
            self.title = title
            self.xLabel = xLabel
            self.yLabel = yLabel
            self.figSize = (figWidth, figWidth / np.sqrt(2))
            self.hasGrid = hasGrid

            self.xData_ = DataLogger()
            self.yData_ = DataLogger()
            self.fig_ = plt.figure(figsize=self.figSize)
            self.ax_ = self.fig_.add_subplot(111)

        def PushPlotXData(self, data: float, legend: str):
            """
            Push data to the buffer

            Args:
                data (float): data to be plotted
            """
            self.xData_.PushData(data, legend)

        def PushPlotYData(self, data: float, legend: str):
            """
            Push data to the buffer

            Args:
                data (float): data to be plotted
            """
            self.yData_.PushData(data, legend)

        def PlotGraph(self):
            """
            Plot the graph
            """
            self.ax_.clear()

            xData = self.xData_.GetData()
            yDataDict = self.yData_.GetData()
            for key, yData in yDataDict.items():
                if xData.get(key) is None:
                    self.ax_.plot(xData[""], yData, label=key)
                else:
                    self.ax_.plot(xData[key], yData, label=key)
            self.ax_.set_title(self.title)
            self.ax_.set_xlabel(self.xLabel)
            self.ax_.set_ylabel(self.yLabel)
            self.ax_.legend()
            if self.hasGrid:
                self.ax_.grid()


    def __init__(self, figWidth = 6, fontName: str = 'Times New Roman',
                 mathFontName = 'stix', fontSize: int = 15, hasGrid: bool = True) -> None:
        """
        Initialize the graph plotter

        Args:
            figWidth (float): width of the figure
            fontName (str): font name
            mathFontName (str): math font name
            fontSize (int): font size
            hasGrid (bool): whether to show grid
        """
        self.figSize_ = (figWidth, figWidth / np.sqrt(2))
        self.fontName_ = fontName
        self.mathFontName_ = mathFontName
        self.fontSize_ = fontSize
        self.hasGrid_ = hasGrid

        self.plots_: dict[str, GraphPlotter.SinglePlot] = {}

        plt.rcParams['figure.figsize'] = self.figSize_
        plt.rcParams['font.family'] = fontName
        plt.rcParams['mathtext.fontset'] = mathFontName
        plt.rcParams['font.size'] = fontSize

    def PushPlotXData(self, data: float, legend: str, title: str = ""):
        """
        Push data to the buffer

        Args:
            data (float): data to be plotted
            legend (str): legend of the data
            title (str): title of the graph
        """
        if title == "":
            for key, plot in self.plots_.items():
                plot.PushPlotXData(data, legend)
            return

        if self.plots_.get(title) is None:
            self.plots_[title] = \
                GraphPlotter.SinglePlot(title, figWidth=self.figSize_[0])
            
        self.plots_[title].PushPlotXData(data, legend)
    
    def PushPlotYData(self, data: float, legend: str, title: str = ""):
        """
        Push data to the buffer

        Args:
            data (float): data to be plotted
            legend (str): legend of the data
            title (str): title of the graph
        """
        if self.plots_.get(title) is None:
            self.plots_[title] = \
                GraphPlotter.SinglePlot(title, figWidth=self.figSize_[0])
            
        self.plots_[title].PushPlotYData(data, legend)

    def SetXLabel(self, xLabel: str, title: str = ""):
        """
        Set x label

        Args:
            xLabel (str): x label
            title (str): title of the graph
        """
        if self.plots_.get(title) is not None:
            self.plots_[title].xLabel = xLabel

    def SetYLabel(self, yLabel: str, title: str = ""):
        """
        Set y label

        Args:
            yLabel (str): y label
            title (str): title of the graph
        """
        if self.plots_.get(title) is not None:
            self.plots_[title].yLabel = yLabel

    def PlotGraphs(self):
        """
        Plot the graphs
        """
        for key, plot in self.plots_.items():
            plot.PlotGraph()

        plt.show()

    def SaveGraphs(self, fileName: str):
        """
        Save the plots to the file

        Args:
            fileName (str): file name
        """
        for key, plot in self.plots_.items():
            plot.PlotGraph()
            fileBaseName, fileExtension = fileName.rsplit('.', 1)
            plot.fig_.savefig(f"{fileBaseName}_{key}.{fileExtension}")

# ----------------------------------------------------------------------------
# * @file GraphPlotter.py
# * History
# * -------
# * - 2024/10/05 New created.(By hoshina)
