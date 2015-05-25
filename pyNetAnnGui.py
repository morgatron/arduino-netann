import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType

app = QtGui.QApplication([])
win = QtGui.QMainWindow()

area = DockArea()
win.setCentralWidget(area)
win.resize(1000,700)
win.setWindowTitle('Application Name!!')

## Create docks, place them into the window one at a time.
## Note that size arguments are only a suggestion; docks will still have to
## fill the entire dock area and obey the limits of their internal widgets.
paramsDock = Dock("Dock1- UI", size=(1, 1))     ## give this dock the minimum possible size
paramsDockA = Dock("Params", size=(300, 500))     ## give this dock the minimum possible size
d2 = Dock("Dock2 - Console", size=(500,200))
d3 = Dock("wave-forms", size=(300,400))
d4 = Dock("Response", size=(500,400))
d5 = Dock("Impedance Graph", size=(500,400))
area.addDock(paramsDock, 'left')      ## place paramsDock at left edge of dock area (it will fill the whole space since there are no other docks yet)
area.addDock(paramsDockA, 'above', paramsDock)
area.addDock(d2, 'right')     ## place d2 at right edge of dock area
area.addDock(d3, 'bottom', paramsDock)## place d3 at bottom edge of paramsDock
area.addDock(d4, 'top', d2)   
area.addDock(d5, 'above', d4)   

#area.moveDock(d4, 'top', d2)     ## move d4 to top edge of d2

## Add widgets into each dock

## first dock gets save/restore buttons
paramsWidget = pg.LayoutWidget()
label = QtGui.QLabel("""Some buttons""")
getParamsBtn = QtGui.QPushButton('Get params')
sendParamsBtn = QtGui.QPushButton('Send params')
#sendParamsBtn.setEnabled(False)

paramsWidget.addWidget(label, row=0, col=0)
paramsWidget.addWidget(getParamsBtn, row=1, col=0)
paramsWidget.addWidget(sendParamsBtn, row=2, col=0)
paramsDock.addWidget(paramsWidget)

scrollWidget = QtGui.QPlainTextEdit()
d2.addWidget(scrollWidget)

#d3.hideTitleBar()
wavePlotWidget = pg.PlotWidget(title="Waveform")
d3.addWidget(wavePlotWidget)
spectrumPlotWidget = pg.PlotWidget(title="Impedance")
responsePlotWidget = pg.PlotWidget(title="Response graph")
d4.addWidget(responsePlotWidget)

paramTree = ParameterTree()
parList = Parameter.create(name='contParams', type='group')
paramsDockA.addWidget(paramTree)

wavePlotWidget.addLegend(offset=[-30,-30])
spectrumPlotWidget.addLegend()
responsePlotWidget.addLegend()
scrollWidget.setReadOnly(True)
scrollWidget.setMaximumBlockCount(500)


#Initialise plots
sens_curve=wavePlotWidget.plot([], pen=None, symbolPen=(255,0,0), symbol='o', name='sense')
heat_curve=wavePlotWidget.plot([], pen=None, symbolBrush=(0,255,0), symbol='o', name='output')
read_curve0F=spectrumPlotWidget.plot([], pen=(255,255,0), name='real')
read_curve90F=spectrumPlotWidget.plot([], pen=(255,0,255), name='imag')
read_curve0=responsePlotWidget.plot([], pen=(255,0,0), name='0')
read_curve90=responsePlotWidget.plot([], pen=(0,255,0), name='90')
read_curveR=responsePlotWidget.plot([], pen=(0,0,255), name='R')
output_curve=responsePlotWidget.plot([], pen=(0,255,255), name='Out')
