# -*- coding: utf-8 -*-
"""
GUI for an arduino temperature controller
"""
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import numpy as np
import scipy as sp
from collections import deque
import pdb
import re, time

import pyqtgdocktest_GUI as gui


Npts=2000
read_data0 = deque([],Npts)
read_data90 = deque([],Npts)
read_dataR = deque([],Npts)
output_data = deque([],Npts)
R_data = deque([],Npts)
C_data = deque([],Npts)
tax = deque([],Npts)

state = None

import time
from pylab import rand
class PretendArd(object):
    t_last=0;
    def flushInput(self):
        pass;
    def flushOutput(self):
        pass;
    def write(self, st):
        pass
    def inWaiting(self):
        t_now=time.time()
        if t_now>self.t_last+0.1:
            self.t_last=t_now
            return True;
        else:
            return False
        
    def readline(self):
        return "d: {0} {1} {2} {3}\n".format(time.time()*1000, int(rand()*1000), int(rand()*1000), 0)

import serial
if 0:
    ardPort=serial.Serial("/dev/ttyACM0", 115200, timeout=0.2, writeTimeout=0.2)
    ardPort.flushInput()
    ardPort.flushOutput()
else:
    ardPort=PretendArd()

contParams=[
    dict(
        name='read_lag',
        type='int',
        suffix='pts',
        limits=(0,100),
        value=0,
        tip='Expected delay for the read waveform (in points)'),
    dict(
        name='sens_phase',
        type='int',
        suffix='pts',
        limits=(0,100),
        value=0,
        tip='Phase of the sense waveform (in points)'
        ),
    dict(
        name='sens_amp',
        type='int',
        limits=(0,4095),
        value=300,
        tip='Size of the sens-waveform (0-4095)'
        ),
    dict(
        name='set_point',
        type='float',
        limits=(0,1e5),
        ),
    dict(
        name='kp',
        type='float',
        value=0.001,
        ),
    dict(
        name='ki',
        type='float',
        value=0.001,
        ),
    dict(
        name='kd',
        type='float',
        value=0.000,
        ),
    dict(
        name='sens_wfm?',
        type='action',
        ),
    dict(
        name='heat_wfm?',
        type='action',
        ),
    dict(
        name='reset_integral',
        type='action',
        ),
]

def sendCommand(cmdName, *args):
    #global scrollWidget # Reference to the scroll area so we can update it
    #print("cmdName is: {}".format(cmdName))
    cmdString=cmdName + ' '+' '.join(args)
    print("sending: {}".format(cmdString))
    gui.scrollWidget.appendPlainText("send -> {}".format(cmdString))
    if ardPort:
        ardPort.write(cmdString+'\n')

def change(param, changes):
    print("tree changes:")
    for param, changeType, newVal in changes:
        #path = p.childPath(param)
        #if path is not None:
            #paramName = '.'.join(path)
        #else:
        paramName = param.name()
        print('  parameter: %s'% paramName)
        print('  changeType:    %s'% changeType)
        print('  data:      %s'% str(newVal))
        print('  ----------')
    if changeType=='value':
        sendCommand(paramName, str(newVal)) 
    if changeType=='activated':
        sendCommand(paramName)#, str(newVal)) 

gui.parList.addChildren(contParams)
gui.parList.sigTreeStateChanged.connect(change)
gui.paramTree.setParameters(gui.parList, showTop=False)

r=re.compile('[: ,]+'); # Regex to match any sequence of colons, 
                #spaces, or commas as seperators in communications from the arduino.

def readAndProcessArd():
    #global scrollWidget; #Scroll area widget
    lines=[]
    #while ardPort.inWaiting():
        
    while ardPort.inWaiting():
        line=ardPort.readline()
        line=line.strip().strip(',')
        line=line.strip()
        lines.append(line)
    for line in lines:
        splts=r.split(line)#.split()
        if len(splts)==0:
            print('Problem with recieved line: "{0}"'.format(line))
            return

        cmdName=splts[0]
        if cmdName != 'd':
            #print('ard-> {}'.format(line))
            gui.scrollWidget.appendPlainText("rec -> {}".format(line))
        args=splts[1:]
        if cmdName=='d':
            updateResponsePlots(*[float(a) for a in args])

        if 1:
            if cmdName=='sens_wfm':
                updateSensWfm(np.array(args, dtype='i4'))
            if cmdName=='heat_wfm':
                updateHeatWfm(np.array(args, dtype='i4'))
            if cmdName in gui.parList.names.keys():
                gui.parList.names[cmdName].setValue(args[0])
                if len(args)>0:
                    print('Too many args ({}) to assign to parameter {}. Will just do the first'.format(args, cmdName))

#k=0
def update():
    #global scrollWidget#,k
    #gui.scrollWidget.appendPlainText(str(k))
    #k+=1
    #updateResponsePlots(0,0,0,0)
    readAndProcessArd()

def updateResponsePlots(sampleTime, read0, read90, output):
    global read_data0, read_data90, output_data, read_curve0, read_curve90, read_curveR, output_curve;

    #read0=sp.rand()
    #read90=sp.rand()
    #output=sp.rand()/2+0.4

    read_data0.append(read0)
    read_data90.append(read90)
    read_dataR.append(sp.sqrt(read0**2+read90**2))
    output_data.append(output)

    #tlast+=0.5+sp.rand()/5 
    tax.append(sampleTime)
    #tax.append(time.time()-tstart)

    gui.read_curve0.setData(tax, read_data0)
    gui.read_curve90.setData(tax, read_data90)
    gui.read_curveR.setData(tax, read_dataR)
    gui.output_curve.setData(tax, output_data)

def updateImpedancePlot(sampleTime, read0, read90, output):
    global R_data, C_data,  R_curve, C_curve
    #read0=sp.rand()
    #read90=sp.rand()
    #output=sp.rand()/2+0.4
    read_data0.append(read0)
    read_data90.append(read90)
    read_dataR.append(sp.sqrt(read0**2+read90**2))
    output_data.append(output)

    #tlast+=0.5+sp.rand()/5 
    tax.append(sampleTime)
    #tax.append(time.time()-tstart)

    gui.read_curve0.setData(tax, read_data0)
    gui.read_curve90.setData(tax, read_data90)
    gui.read_curveR.setData(tax, read_dataR)
    gui.output_curve.setData(tax, output_data)

def updateSensWfm(dat):
    gui.sens_curve.setData(dat)
def updateHeatWfm(dat):
    gui.heat_curve.setData(dat)

def sendParams():
    print("Sending params to arudino") 
    print("Not implemented yet")
    #Somehow loop through a list of parameters and send them all
def getParams():
    print("Get params from arduino")
    print("Not implemented yet")


gui.getParamsBtn.clicked.connect(getParams)
gui.sendParamsBtn.clicked.connect(sendParams)

timer = QtCore.QTimer()
timer.timeout.connect(update)



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    timer.start(50)
    gui.win.show()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
