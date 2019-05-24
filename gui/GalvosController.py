# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 10:48:31 2015

@author: flesage
"""
import pyqtgraph as pg

from datetime import date
import time
import math
import os
from main import config
from PyQt5 import QtGui
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QPushButton
from PyQt5.QtGui import QIcon
from scipy import signal
from base.Maitai import Maitai

from base import liomio
from base.liomacq import OnDemandVoltageOutTask
from base.po2Acq import PO2Acq
import numpy as np
import icons_rc
from base.liomio import DataSaver
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import posixpath
from gui.ImageDisplay import po2Viewer
from base.Motors import ThorlabsMotor

import ImageDisplay as imdisp
from scipy.interpolate import interp1d
import h5py

import matplotlib
matplotlib.use("Qt5Agg") #new line
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import threading
'''
from scipy import ndimage
import zaber.serial
from base.Maitai import InsightDS
from base.Motors import ThorlabsMotor
import nibabel as nb
from base.liomio import NiiStackSaver
import os
import threading
'''

class GalvosController(QWidget):
    
    changedOffsetX = pyqtSignal(int)
    changedOffsetY = pyqtSignal(int)
    changedOffsetDisplay = pyqtSignal(int)
    
    
    def __init__(self):
        QWidget.__init__(self)
        basepath= os.path.join(os.path.dirname(__file__))
        uic.loadUi(os.path.join(basepath,"galvos_form_NEW.ui"), self)

        # Mechanical shutters
        self.shutter2ph_closed = True                      
        self.shutter3ph_closed = True                      
        self.pushButton_shutter_2ph.clicked.connect(self.toggle_shutter2ph)
        self.pushButton_shutter_3ph.clicked.connect(self.toggle_shutter3ph)
        self.pushButton_shutter_2ph.setStyleSheet("background-color: pale gray");
        self.pushButton_shutter_3ph.setStyleSheet("background-color: pale gray");

        self.maitaiFlag=0
        self.checkBoxMaiTai.clicked.connect(self.toggle_maitai_control)
        self.pushButton_maitai.setEnabled(False)
        self.pushButton_maitaiShutter.setEnabled(False)
        self.pushButton_maitai.setEnabled(False)
        self.pushButton_maitaiReadPower.setEnabled(False)
        self.pushButton_maitaiReadWavelength.setEnabled(False)
        self.pushButton_maitaiSetWavelength.setEnabled(False)
        self.pushButton_maitaiCheckStatus.setEnabled(False)
        self.pushButton_maitaiML.setEnabled(False)
        
        # Laser Maitai control
        self.maitai_off = True
        self.pushButton_maitai.clicked.connect(self.toggle_maitai)
        self.maitaiShutter_closed = True        
        self.pushButton_maitaiShutter.clicked.connect(self.toggle_maitaiShutter)
        self.pushButton_maitai.setStyleSheet("background-color: pale gray");
        self.pushButton_maitaiShutter.setStyleSheet("background-color: pale gray");
        self.pushButton_maitaiShutter.setEnabled(False)
        self.pushButton_maitaiReadPower.clicked.connect(self.readPower_maitai)
        self.pushButton_maitaiReadPower.setStyleSheet("background-color: pale gray");   
        self.pushButton_maitaiReadPower.setEnabled(False)  
        self.pushButton_maitaiReadWavelength.clicked.connect(self.readWavelength_maitai)
        self.pushButton_maitaiReadWavelength.setStyleSheet("background-color: pale gray");
        self.pushButton_maitaiReadWavelength.setEnabled(False)
        self.pushButton_maitaiSetWavelength.clicked.connect(self.setWavelength_maitai)
        self.pushButton_maitaiSetWavelength.setStyleSheet("background-color: pale gray");
        self.pushButton_maitaiSetWavelength.setEnabled(False)
        self.pushButton_maitaiCheckStatus.setStyleSheet("background-color: pale gray");
        self.pushButton_maitaiCheckStatus.clicked.connect(self.readStatus_maitai)
        self.pushButton_maitaiCheckStatus.setEnabled(False)
        self.pushButton_maitaiML.setEnabled(False)
        
        # Laser power
        self.horizontalScrollBar_power2ph.valueChanged.connect(self.setPower2ph)
        self.horizontalScrollBar_power3ph.valueChanged.connect(self.setPower3ph)
        self.pushButton_set_3Ph_power_minimum.clicked.connect(self.setMinimumPower3ph)
        self.pushButton_set_3Ph_power_maximum.clicked.connect(self.setMaximumPower3ph)
        self.pushButton_reset_3Ph_wheel.clicked.connect(self.resetWheel3ph)
        self.wheel3PMax=360.0

        self.wheel3P_angle_offset=0 #offset of 10 degrees
        
        # Galvos
        validator1=QtGui.QIntValidator(1,5000,self)
        self.lineEdit_nx.setValidator(validator1)
        self.lineEdit_ny.setValidator(validator1)
        self.saving = None
        validator2=QtGui.QIntValidator(1,4000,self)
        self.lineEdit_width.setValidator(validator2)
        self.lineEdit_height.setValidator(validator2)        
        validator3=QtGui.QIntValidator(1,500,self)
        self.lineEdit_extrapoints.setValidator(validator3)
        validator4=QtGui.QIntValidator(1,1000,self)
        self.lineEdit_linerate.setValidator(validator4)
        self.comboBox_scantype.activated.connect(self.scantype_chosen)
        self.pushButton_start.clicked.connect(self.startscan)
        self.pushButton_stop.clicked.connect(self.stopscan)   
        
        self.galvos_stopped = True
        self.stopped = False
        
        # Motors X,Y,Z
        self.pushButton_center.clicked.connect(self.move_center)
        self.pushButton_down.clicked.connect(self.move_down)
        self.pushButton_right.clicked.connect(self.move_right)
        self.pushButton_up.clicked.connect(self.move_up)
        self.pushButton_left.clicked.connect(self.move_left)
        self.pushButton_set_brain.clicked.connect(self.set_brain_pos)  
        self.pushButton_goto_brain.clicked.connect(self.goto_brain_pos)        
        self.pushButton_goto_brain.setEnabled(False)            
        self.pushButton_motor_up.clicked.connect(self.move_z_up)
        self.pushButton_motor_down.clicked.connect(self.move_z_down)
        self.pushButton_set_center.clicked.connect(self.set_xy_center)
                
        self.microstepsize=0.49609375
        self.pushButton_center.setEnabled(False)
        self.brainPosSetFlag = 0
        
        #Stack acquisition:
        self.pushButton_start_stack.clicked.connect(self.start_stack_thread)
        self.pushButton_stop_stack.clicked.connect(self.stop_stack)   
        self.pushButton_stop.setEnabled(False)
        self.pushButton_stop_stack.setEnabled(False)
        
        self.galvos=None
        self.center_x=0.0
        self.center_y=0.0
        self.t1 = None
        self.topleft= None
        self.bottomright= None
        self.power_motor = None
        
        self.scanDate = str(date.today())
        self.liveScanNumber=0
        self.stackNumber=0
        
        #Line scan acquistion:
        self.toggleLineFlag=0;
        self.pushButton_get_line_position.clicked.connect(self.update_linescan)
        self.lineEdit_nt.textChanged.connect(self.update_linescan)
        self.lineEdit_linerate_LS.textChanged.connect(self.update_linescan)
        self.pushButton_snap_angio.setEnabled(False)        
        self.pushButton_start_linescan.setEnabled(True)
        self.pushButton_stop_linescan.setEnabled(False)
        self.pushButton_snap_angio.clicked.connect(self.take_snapshot)
        self.horizontalScrollBar_line_scan_shift_y.setValue(-20)
        self.horizontalScrollBar_line_scan_shift_x.setValue(0)
        self.horizontalScrollBar_line_scan_shift_display.setValue(106)
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.update_linescan)
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.update_linescan)
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.shift_x_changed_value)
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.shift_y_changed_value)
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.shift_display_changed_value)
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.update_linescan)
        self.pushButton_start_linescan.clicked.connect(self.startlinescan)
        self.pushButton_stop_linescan.clicked.connect(self.stoplinescan)   
        self.previewScanFlag = False
        self.linescan_shift_display=0
        
        # PO2 acquisition
        self.po2_x_positions = None
        self.po2_y_positions = None
        self.pushButton_startPO2.clicked.connect(self.start_po2_scan)
        self.pushButton_po2AddPoint.clicked.connect(self.po2_add_point)
        
        #Channel 1 - 2: display
        self.pushButton_Channel_1.clicked.connect(self.showChannel1)
        self.pushButton_Channel_2.clicked.connect(self.showChannel2)
        
        #Power curve:
        self.positionVector=[]
        self.powerVector=[]
        self.pushButton_add_power_val.clicked.connect(self.addPowerValue)
        self.pushButton_clear_curve.clicked.connect(self.clearCurve)
        self.checkBoxPowerCurve.setEnabled(False)
        self.pushButton_add_power_val.setEnabled(False)
        self.pushButton_show_curve.clicked.connect(self.showPowerCurve)
        self.pushButton_show_curve.setEnabled(False)

        #Power curve 3P:
        self.positionVector3P=[]
        self.powerVector3P=[]
        self.pushButton_add_power_val_3P.clicked.connect(self.addPowerValue3P)
        self.pushButton_clear_curve_3P.clicked.connect(self.clearCurve3P)
        self.pushButton_add_power_val_3P.setEnabled(False)
        self.pushButton_show_curve3P.clicked.connect(self.showPowerCurve3P)
        self.pushButton_show_curve3P.setEnabled(False)
        self.checkBoxPowerCurve3P.setEnabled(False)

        #Saving:
        self.pushButton_select_save_directory.clicked.connect(self.select_saving_directory)
        self.lineEdit_mouse_name.setEnabled(False)
        self.checkBox_enable_save.setEnabled(False)

        #Save folder:
        self.pushButton_set_save_name.setEnabled(False)
        self.lineEdit_save_name.setEnabled(False)
        self.pushButton_set_folder.clicked.connect(self.set_save_folder)
        self.pushButton_set_save_name.clicked.connect(self.create_new_folder)
        
        # 3P Wheel:
        self.pushButton_3PWheel_Activate.clicked.connect(self.toggle3PWheel)
        self.enableWheelFlag = 0
        self.flagWheel = 1
        
        #Multiple lines:
        self.pushButtonAddLine.clicked.connect(self.addLines)
        self.pushButtonShowLines.clicked.connect(self.displayLines)
        self.toggleDisplayLines = 1
        #self.pushButtonStartMultipleLineAcq.clicked.connect(self.start_multiple_lines)
        #self.pushButtonStopMultipleLineAcq.clicked.connect(self.stop_multiple_lines)
        self.pushButtonStopMultipleLineAcq.setEnabled(False)
        self.pushButtonStartMultipleLineAcq.setEnabled(False)
        self.pushButtonShowLines.setEnabled(False)

        self.pushButtonStartMultipleLineAcq.clicked.connect(self.startlinescanTimer)
        self.pushButtonStopMultipleLineAcq.clicked.connect(self.abortlinescanTimer)
        
        #preview:
        self.pushButton_preview.clicked.connect(self.changeDisplayValues)
        self.previewFlag=0;
        
        self.runCalX=False
        self.runCalY=False
        
        #PO2:
        self.pushButton_defineROI.clicked.connect(self.defineROI_PO2)
        self.pushButton_RemovePoints.clicked.connect(self.clearGrid_PO2)
        self.numberOfPoints=0
        self.horizontalScrollBar_xPoints.setValue(1)
        self.horizontalScrollBar_yPoints.setValue(1)
        self.horizontalScrollBar_xPoints.valueChanged.connect(self.modifyGrid_PO2)
        self.horizontalScrollBar_yPoints.valueChanged.connect(self.modifyGrid_PO2)
        self.numberOfXPoints=3
        self.numberOfYPoints=4
        
        self.po2_add_point_state=True
        self.tabWidget.setCurrentIndex(1)  
        
    #Wheel 3P:
    def toggle3PWheel(self):
        if (self.enableWheelFlag==0):
            print('Initialising 3P rotative motor...')
            self.thorlabs = ThorlabsMotor(self.thorlabsWheelSN, self.thorlabsWheelHW)
            status=True
            self.enableWheelFlag=1
            self.pushButton_3PWheel_Activate.setEnabled(False)
            #self.pushButton_3PWheel_Activate.setStyleSheet("background-color: red");
            #self.pushButton_3PWheel_Activate.setText('Disable Wheel')            
            print('...done!')
        #else:
        #    self.pushButton_3PWheel_Activate.setStyleSheet("background-color: pale gray");
        #    self.pushButton_3PWheel_Activate.setText('Activate Wheel')     
        #    print('Killing 3P Motor...')
        #    self.turnOffWheel3P()
        #    self.thorlabs.clean_up_APT()
        #    status=False
        #    self.enableWheelFlag=0
        #    print('...done!')
        self.pushButton_set_3Ph_power_minimum.setEnabled(status)
        self.pushButton_set_3Ph_power_maximum.setEnabled(status)
        self.pushButton_reset_3Ph_wheel.setEnabled(status)
        self.horizontalScrollBar_power3ph.setEnabled(status)
        self.label_3PhWheel.setEnabled(status)
        self.label_power3P.setEnabled(status)
        
    def kill3PWheel(self):
        if (self.enableWheelFlag==1):
            print('Killing 3P Motor...')
            self.thorlabs.clean_up_APT()
            self.turnOffWheel3P()
            print('...done!')

        
    def getInfoThorlabsWheel(self,SN,HW):
        self.thorlabsWheelSN=SN
        self.thorlabsWheelHW=HW
    
        
    #PO2 functons:
    
    def updateViewerConsumer(self,flag):
        self.ai_task.updateConsumerFlag('viewer',flag)
    
    def po2_add_point(self):
        self.toggle_po2_add_point_state()
        
    def toggle_po2_add_point_state(self):
        if (self.po2_add_point_state):
            self.pushButton_po2AddPoint.setText('Stop adding points')
            self.po2_add_point_state=False
            self.viewer.updateGeneratePointFlag(True)
        else:
            self.pushButton_po2AddPoint.setText('Add Points')
            self.po2_add_point_state=True
            self.viewer.updateGeneratePointFlag(False)
    
    def defineROI_PO2(self):
        self.viewer.resetRect()
        nx=float(self.lineEdit_nx.text())
        ny=float(self.lineEdit_ny.text())
        self.viewer.createRectangle(nx,ny)
        self.viewer.displayRectangles()
    
    def generateGrid_PO2(self):
        [x_o,y_o,width,height]=self.viewer.getMouseSelectedRectPosition()
        self.nx=float(self.lineEdit_nx.text())
        self.ny=float(self.lineEdit_ny.text())
        self.n_extra=float(self.lineEdit_extrapoints.text())
        self.width=float(self.lineEdit_width.text())
        self.height=float(self.lineEdit_height.text())
        
        if (x_o!=0 and (y_o!=0) and (width!=0) and (height)):
            self.oldNumberOfPoints=self.numberOfYPoints*self.numberOfXPoints
            x_pos=(np.linspace(x_o, x_o+width, num=self.numberOfXPoints))
            y_pos=(np.linspace(y_o, y_o+height, num=self.numberOfYPoints))
            y_pos_grid=np.zeros(x_pos.shape[0]*y_pos.shape[0])
            x_pos_grid=np.zeros(x_pos.shape[0]*y_pos.shape[0])
            counter=0
            for i in range(x_pos.shape[0]):
                for j in range(y_pos.shape[0]):
                    y_pos_grid[counter]=(y_pos[j]-self.ny/2)*self.height/self.ny
                    x_pos_grid[counter]=(x_pos[i]-self.nx/2)*self.width/self.nx
                    counter=counter+1
            x_pos_grid.astype(int)
            y_pos_grid.astype(int)
            self.po2_x_positions = x_pos_grid
            self.po2_y_positions = y_pos_grid
            for i in x_pos:
                for j in y_pos:
                    self.viewer.createPoint(i,j)
            self.viewer.displayPoints()
            self.viewer.resetRect()
            
    def convertPosToVolts(self):
        self.nx=float(self.lineEdit_nx.text())
        self.ny=float(self.lineEdit_ny.text())
        self.n_extra=float(self.lineEdit_extrapoints.text())
        self.width=float(self.lineEdit_width.text())
        self.height=float(self.lineEdit_height.text())
         

        [self.x_pos,self.y_pos]=self.viewer.getPositionPoints()
        counter=0
        lengthx = len(self.x_pos)
        x_pos_grid=np.zeros(lengthx)
        y_pos_grid=np.zeros(lengthx)
        for i in range(lengthx):
            y_pos_grid[counter]=(self.y_pos[i]-self.ny/2)*self.height/self.ny
            x_pos_grid[counter]=(self.x_pos[i]-self.nx/2)*self.width/self.nx
            counter=counter+1
        x_pos_grid.astype(int)
        y_pos_grid.astype(int)
        self.po2_x_positions = x_pos_grid
        self.po2_y_positions = y_pos_grid

    def modifyGrid_PO2(self):
        self.viewer.removeAllPoints()
        self.numberOfXPoints=self.horizontalScrollBar_xPoints.value()
        self.numberOfYPoints=self.horizontalScrollBar_yPoints.value()
        self.lineEdit_numXpoints.setText(str(self.numberOfXPoints))
        self.lineEdit_numYpoints.setText(str(self.numberOfYPoints))
        self.generateGrid_PO2()
        
    def clearGrid_PO2(self):
        self.viewer.removeAllPoints()
        self.viewer.resetRect()
        
    def start_po2_scan(self):
        self.convertPosToVolts()
        self.showPO2Viewer()
        self.ai_task.updateConsumerFlag('viewer',False)        
        
        power2ph=self.horizontalScrollBar_power2ph.value()
        self.toggle_shutter2ph()
        print('Start po2 acquisition:')
        # Fermer EOM manuel
        try:
            self.power_ao_eom.ClearTask()
        except AttributeError:
            print('eom already cleared!')
        else:
            del(self.power_ao_eom)

            
        # Steps
        gate_on = float(self.lineEdit_po2_gate_on.text())
        gate_off = float(self.lineEdit_po2_gate_off.text())
        voltage_on = 2.0*power2ph/100.0
        n_averages = int(self.lineEdit_n_po2_averages.text())
        
        eom_task = PO2Acq(config.eom_device, config.eom_ao, gate_on, gate_off, voltage_on, n_averages)
        eom_task.setSynchronizedAITask(self.ai_task)
        eom_task.config()
        # Acquire list of points to move to
        self.galvos.configOnDemand()
        self.ai_task.setDecoder(None)
        self.ai_task.setDataConsumer(self.po2viewer,True,0,'po2plot',True)

        # Loop over points and:
        for i in range(self.po2_x_positions.shape[0]):
            
            print('po2 measurements: ' + str(i+1) + ' out of ' + str(self.po2_x_positions.shape[0]))
            self.galvos.moveOnDemand(self.po2_x_positions[i], self.po2_y_positions[i])
            print('galvos moved...')
            eom_task.start()
            # One second seems too much if you want to go faster
            time.sleep(1)
            print(str(self.po2_x_positions[i])+';'+str(self.po2_y_positions[i]))
            self.po2viewer.update()

        #    Start acquisition task averaging doing a finite ao task reapeated n average times
        #    Show in a plot the Decay curve
        self.toggle_shutter2ph()
        eom_task.close()
        self.galvos.ao_task.StopTask()
        self.galvos.ao_task.ClearTask()
        self.ai_task.setDecoder(self.galvos)
        #self.viewer.removeAllPoints()
        # Remettre EOM Manuel
        self.power_ao_eom = OnDemandVoltageOutTask(config.eom_device, config.eom_ao, 'Power2Ph')
        print('po2 acquisition done!')
        self.ai_task.updateConsumerFlag('viewer',True)        
        self.ai_task.removeDataConsumer(self.po2viewer)
        timer.stop()

    #PREVIEW VALUES
    def changeDisplayValues(self):
        if self.previewFlag==0:
            self.nxTemp=self.lineEdit_nx.text()
            self.nyTemp=self.lineEdit_ny.text()
            self.linerateTemp=self.lineEdit_linerate.text()    
            self.lineEdit_nx.setText('256')
            self.lineEdit_ny.setText('256')
            self.lineEdit_linerate.setText('300')            
            self.pushButton_preview.setText('Undo')
            self.previewFlag=1
        else: 
            self.lineEdit_nx.setText(self.nxTemp)
            self.lineEdit_ny.setText(self.nxTemp)
            self.lineEdit_linerate.setText(self.linerateTemp)            
            self.pushButton_preview.setText('Preview')
            self.previewFlag=0
            
        
    #IMAGE VIEWER FUNCTIONS:    
        
    def setImageViewer(self,viewer):
        self.viewer = viewer
    
    def setImageViewer2(self,viewer2):
        self.viewer2 = viewer2
 
    def showChannel1(self):
        self.viewer.showWindow()    

    def showChannel2(self):
        self.viewer2.showWindow() 
 
    def setPO2Viewer(self,viewer):
        self.po2viewer=viewer
        
    def showPO2Viewer(self):
        self.po2viewer.showPlot()
    #MULTIPLE LINE FUNCTIONS:
    
    def addLines(self):
        self.viewer.addLines()
        self.updateNumberOfLines()

    def removeSelectedLine(self):
        self.viewer.removeSelectedLine()
        self.updateNumberOfLines()
        
    def removeLastLine(self):
        self.viewer.removeLastLine()
        self.updateNumberOfLines()
        
    def hideLines(self):
        self.viewer.resetLines()                
        self.pushButtonShowLines.setText('Show Lines')      
        self.toggleDisplayLines = 1        
        
    def displayLines(self):
        if self.toggleDisplayLines == 1:
            self.viewer.displayLines()      
            self.pushButtonShowLines.setText('Hide Lines')  
            self.toggleDisplayLines = 0   
        else:    
            self.viewer.resetLines()                
            self.pushButtonShowLines.setText('Show Lines')      
            self.toggleDisplayLines = 1
    
    def updateNumberOfLines(self):
        self.numberOfLines=len(self.viewer.lines)  
        txt = 'Number of lines: '+ str(self.numberOfLines)
        self.label_numberOfLines.setText(txt)
        if self.numberOfLines>0:
            self.pushButtonStartMultipleLineAcq.setEnabled(True)
            self.pushButtonShowLines.setEnabled(True)
        else:
            self.pushButtonStartMultipleLineAcq.setEnabled(False)
            self.pushButtonShowLines.setEnabled(False)
        
    #POWER CURVE:    
        
    def togglePowerCurve(self,status):
        self.checkBoxPowerCurve.setEnabled(status)
        
    def toggleShowCurve(self,status):
        self.pushButton_show_curve.setEnabled(status)



    def clearCurve(self):
        self.positionVector=[]
        self.powerVector=[]
        self.togglePowerCurve(False)
        self.toggleShowCurve(False)
       
    def addPowerValue(self):
        self.get_current_z_depth()
        currentPowerValue=self.horizontalScrollBar_power2ph.value()
        
        self.positionVector.append(self.currentZPos)
        self.powerVector.append(currentPowerValue)
        
        self.lineEdit_current_depth.setText(str(self.currentZPos))
        txt= str(currentPowerValue) + '%'
        self.lineEdit_current_power.setText(txt)
        
        if len(self.positionVector)==4:
            self.togglePowerCurve(True)
            self.toggleShowCurve(True)


    def generatePowerCurve(self):
        self.positionVectorArray = np.array(self.positionVector)
        self.powerVectorArray = np.array(self.powerVector).astype(float)
        self.zstep=float(self.lineEdit_zstep.text())
        self.depthVector=np.arange(0,1000.0,self.zstep)/1000.0
        
        f2 = interp1d(self.positionVectorArray, self.powerVectorArray, kind='quadratic',fill_value='extrapolate')
        self.powerCurve=f2(self.depthVector)
        self.powerCurve[self.powerCurve >= 100] = 100
        self.powerCurve[self.powerCurve <= 0] = 0
        firstindex=np.searchsorted(self.powerCurve, 100)
        indices=np.arange(firstindex,len(self.powerCurve),1)
        vals=100*np.ones(len(indices))
        self.powerCurve[indices]=vals
        
        
    def showPowerCurve(self):
        self.TwoPplot = pg.PlotWidget(None)
        self.TwoPplot.setWindowTitle('2P power curve')
        self.TwoPplot.plot(title='2P power curve')
        self.TwoPplot.setTitle('2P power curve')
        self.TwoPplot.setLabel('left', text='Intensity')
        self.TwoPplot.setLabel('bottom', text='Depth')
        self.TwoPplot.resize(650, 450)
        self.TwoPplot.move(1200, 500)  
        self.TwoPplot.show()
        self.generatePowerCurve()
        self.TwoPplot.plot(self.depthVector, self.powerCurve)
        #self.TwoPplot.setData(self.positionVector,self.powerVector)#,pen=None, symbol='o', symbolPen=None, symbolSize=4, symbolBrush=('r'))
        scatter = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='o', size=1)
        self.TwoPplot.addItem(scatter)
        scatter.setData(self.positionVector,self.powerVector)

        
        #fig, ax = plt.subplots()        
        #self.generatePowerCurve()
        #ax.plot(self.depthVector, self.powerCurve)
        #ax.plot(self.positionVector,self.powerVector,linestyle="None", marker='o', color='r')
        #ax.grid(True)
        #plt.xlabel('axial position [mm]')
        #plt.ylabel('power [%]')
        #plt.title('Power curve 2P')
        #plt.ylim(0,110)
        #plt.show()

    #POWER CURVE:    
        
    def togglePowerCurve3P(self,status):
        self.checkBoxPowerCurve3P.setEnabled(status)

    def toggleShowCurve3P(self,status):
        self.pushButton_show_curve3P.setEnabled(status)

    def clearCurve3P(self):
        self.positionVector3P=[]
        self.powerVector3P=[]
        self.togglePowerCurve3P(False)
        self.toggleShowCurve3P(False)

        
    def addPowerValue3P(self):
        self.get_current_z_depth()
        currentPowerValue3P=self.horizontalScrollBar_power3ph.value()
        
        self.positionVector3P.append(self.currentZPos)
        self.powerVector3P.append(currentPowerValue3P)
        
        self.lineEdit_current_depth_3P.setText(str(self.currentZPos))
        txt= str(currentPowerValue3P) + '%'
        self.lineEdit_current_power_3P.setText(txt)
        
        if len(self.positionVector3P)==4:
            self.togglePowerCurve3P(True)
            self.toggleShowCurve3P(True)

    def generatePowerCurve3P(self):
        self.positionVector3Parray = np.array(self.positionVector3P)
        self.powerVector3Parray = np.array(self.powerVector3P).astype(float)
        self.zstep=float(self.lineEdit_zstep.text())
        self.depthVector3P=np.arange(0,1000.0,self.zstep)/1000.0
        
        f2 = interp1d(self.positionVector3Parray, self.powerVector3Parray, kind='cubic',fill_value='extrapolate')
        self.powerCurve3P=f2(self.depthVector3P)
        self.powerCurve3P[self.powerCurve3P >= 100] = 100

    def showPowerCurve3P(self):  
        fig, ax = plt.subplots()        
        self.generatePowerCurve3P()
        ax.plot(self.depthVector3P, self.powerCurve3P)
        ax.plot(self.positionVector3P,self.powerVector3P,linestyle="None", marker='o', color='r')
        ax.grid(True)
        plt.xlabel('axial position [mm]')
        plt.ylabel('power [%]')
        plt.title('Power curve 3P')
        plt.ylim(0,110)
        plt.show()                
    #LINE SCAN FUNCTIONS:

    def toggle_line(self):
        self.viewer.toggleLinearSelection()
        if self.toggleLineFlag==0:
            self.toggleLineFlag=1
            self.toggle_linescanDisplayButton(False)
        else:        
            self.toggle_linescanDisplayButton(True)
            self.toggleLineFlag=0
            
    def toggle_linescanDisplayButton(self,val):
        self.pushButton_start_stack.setEnabled(val)
        self.pushButton_stop_stack.setEnabled(val)  
              
        if self.pushButton_stop.isEnabled():
            self.pushButton_stop.setEnabled(True)
        else:
            self.pushButton_stop.setEnabled(val)
            
        if self.pushButton_start.isEnabled():
            self.pushButton_start.setEnabled(False)
        else:
            self.pushButton_start.setEnabled(val)
                        
        val = (val == False)
        self.pushButton_snap_angio.setEnabled(val)        
        self.pushButton_start_linescan.setEnabled(val)
        self.pushButton_get_line_position.setEnabled(val)
            
    def update_linescan(self):
        linescan_width, center_x, center_y=self.get_line_pos()
        self.lineEdit_line_scan_width.setText(str(round(linescan_width,2)))
                 
        n_lines=float(self.lineEdit_nt.text())   
        line_rate=float(self.lineEdit_linerate_LS.text())     
        linescan_totalTime=n_lines/line_rate
        self.lineEdit_line_scan_time.setText(str(linescan_totalTime))        
         
        linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value()) 
        self.lineEdit_linescan_shift_x.setText(str(linescan_shift_x))
        linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value()) 
        self.lineEdit_linescan_shift_y.setText(str(linescan_shift_y)) 
        self.linescan_shift_display=int(self.horizontalScrollBar_line_scan_shift_display.value()) 
        self.lineEdit_linescan_shift_display.setText(str(self.linescan_shift_display)) 
         
        self.lineEdit_CentralPosition.setText(str(round(center_x,2))+' / '+str(round(center_y,2)))      

        
    def get_line_pos(self):
        print('in get line position')
        linescan_px_x_1, linescan_px_y_1, linescan_px_x_2, linescan_px_y_2, linescan_length_px = self.viewer.getMouseSelectedLinePosition()
        print(str(linescan_px_x_1))
        nx=int(self.lineEdit_nx.text())   
        ny=int(self.lineEdit_ny.text())   
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())      
        
        linescan_x_1_um=linescan_px_x_1/nx*width
        linescan_x_2_um=linescan_px_x_2/nx*width
        linescan_y_1_um=linescan_px_y_1/ny*height
        linescan_y_2_um=linescan_px_y_2/ny*height
        
        center_x=(linescan_x_2_um+linescan_x_1_um)/2-width/2
        center_y=(linescan_y_2_um+linescan_y_1_um)/2-height/2
        
        linescan_width = math.sqrt(math.pow((linescan_x_2_um-linescan_x_1_um),2)+math.pow((linescan_y_2_um-linescan_y_1_um),2))
        
        strCoord=str(center_x)+'/'+str(center_y)
            
        txtCoord = open(r"C:\git-projects\multiphoton\coordinates.txt","w") 
        txtCoord.write(strCoord)
        txtCoord.close()
        
        return linescan_width, center_x, center_y
   
    def shift_x_changed_value(self, value):
        print('in x value changed function')
        self.changedOffsetX.emit(value)

    def shift_y_changed_value(self, value):
        print('in y value changed function')
        self.changedOffsetY.emit(value)
        
    def shift_display_changed_value(self, value):
        print('in display value changed function')
        self.changedOffsetDisplay.emit(value)
        
    #LASERS FUNCTIONS:

    def toggle_maitai_control(self):
        if (self.maitaiFlag==0):
            maitaiStatus=True
            self.maitaiFlag=1
            self.lineEdit_maitai_status.setText('Connecting to MaiTai...')
            laser = Maitai(config.comPortLaser)    
            replaser = laser.ReadStatus()
            self.lineEdit_maitai_status.setText('Status laser:' + replaser)
            self.setMaitai(laser)
        else:
            maitaiStatus=False
            self.maitaiFlag=0    
            self.lineEdit_maitai_status.setText('Closing connection to MaiTai...')
            self.maitai.CloseLaser()
            self.maitai.serial.close()
            del self.maitai
            self.lineEdit_maitai_status.setText('Closing connection to MaiTai... done!')

            
        self.pushButton_maitai.setEnabled(maitaiStatus)
        self.pushButton_maitaiShutter.setEnabled(maitaiStatus)
        self.pushButton_maitai.setEnabled(maitaiStatus)
        self.pushButton_maitaiReadPower.setEnabled(maitaiStatus)
        self.pushButton_maitaiReadWavelength.setEnabled(maitaiStatus)
        self.pushButton_maitaiSetWavelength.setEnabled(maitaiStatus)
        self.pushButton_maitaiCheckStatus.setEnabled(maitaiStatus)
        self.pushButton_maitaiML.setEnabled(maitaiStatus)

    def closeLaser(self):
        if (self.maitaiFlag==1):
            self.maitai.CloseLaser()
            self.maitai.SHUTTERshut()
            
    def setMaitai(self,maitai):
        self.maitai = maitai
        
    def toggle_maitai(self):
        if self.maitai_off:
            print 'Turning laser on...'
            self.maitai.OpenLaser()
            warmup_response=self.maitai.WarmedUp()
            warmup_str = "Warmup status:" + warmup_response
            if (int(warmup_response) == 0):
                warmup_str = warmup_str + ' - Begin'
            elif (int(warmup_response) == 100):
                warmup_str = warmup_str + ' - Diode laser turning on'
            else:
                warmup_str = warmup_str + ' - Error'
            
            self.lineEdit_laserResponse.setText(warmup_str)
            self.maitai_off = False
            self.pushButton_maitai.setStyleSheet("background-color: yellow");
            self.pushButton_maitai.setText('Maitai : ON')
            self.pushButton_maitaiShutter.setEnabled(True)
            self.pushButton_maitaiReadPower.setEnabled(True)
            self.pushButton_maitaiReadWavelength.setEnabled(True)
            self.pushButton_maitaiSetWavelength.setEnabled(True)   
            self.pushButton_maitaiCheckStatus.setEnabled(True)
                     
        else:
            print 'Turning laser off...'
            self.maitai.CloseLaser() 
            self.maitai_off = True
            self.pushButton_maitai.setStyleSheet("background-color: pale gray");
            self.pushButton_maitai.setText('Maitai : Off')
            self.maitai.SHUTTERshut() 
            self.maitaiShutter_closed = True
            self.pushButton_maitaiShutter.setEnabled(False)
            self.pushButton_maitaiReadPower.setEnabled(False)
            self.pushButton_maitaiReadWavelength.setEnabled(False)            
            self.pushButton_maitaiSetWavelength.setEnabled(False)     
            self.pushButton_maitaiCheckStatus.setEnabled(False)
                
    def toggle_maitaiShutter(self):
        if self.maitaiShutter_closed:
            print 'Laser shutter open...'
            self.maitai.SHUTTERopen()
            self.maitaiShutter_closed = False
            self.pushButton_maitaiShutter.setStyleSheet("background-color: red");
            self.pushButton_maitaiShutter.setText('Maitai Shutter: OPEN')
        else:
            print 'Laser shutter closed...'
            self.maitai.SHUTTERshut() 
            self.maitaiShutter_closed = True
            self.pushButton_maitaiShutter.setStyleSheet("background-color: pale gray");
            self.pushButton_maitaiShutter.setText('Maitai Shutter: Closed')  
            
    def readPower_maitai(self):
        print 'Reading laser power...'
        powerMaitai = self.maitai.ReadPower()
        powerMaitai = str(round(float(powerMaitai[0:-2])*100.0)/100.0)
        self.lineEdit_laserResponse.setText("laser power: "+powerMaitai+"W")
        
    def checkPowerStatus_maitai(self):
        print 'Checking laser status...'
        powerMaitai = self.maitai.ReadPower()
        powerMaitai = powerMaitai[0:-2]
        powerMaitai = str(round(float(powerMaitai)*100.0)/100.0)
        self.lineEdit_laserResponse.setText("laser power: "+powerMaitai+'W')
        statusMaitai = self.maitai.ReadStatus()
        if(int(statusMaitai)==13):
            self.pushButton_maitaiML.setStyleSheet("background-color: red"); 
        elif(int(statusMaitai)==15):
            self.pushButton_maitaiML.setStyleSheet("background-color: green"); 
        else:
            self.pushButton_maitaiML.setStyleSheet("background-color: pale gray"); 
                    
    def readCurrent_maitai(self):
        currentMaitai = self.maitai.ReadCurrent()
        print 'Laser diode current: ' + currentMaitai +' %'

    def readStatus_maitai(self):
        print 'Reading laser status...'
        statusMaitai = self.maitai.ReadStatus()
        self.lineEdit_laserResponse.setText("laser status: "+statusMaitai)   
        if(int(statusMaitai)==13):
            self.pushButton_maitaiML.setStyleSheet("background-color: red"); 
        elif(int(statusMaitai)==15):
            self.pushButton_maitaiML.setStyleSheet("background-color: green"); 
        else:
            self.pushButton_maitaiML.setStyleSheet("background-color: pale gray"); 
        
    def readWavelength_maitai(self):
        print 'Reading laser wavelength...'
        wavelengthMaitai = self.maitai.ReadWavelength()
        self.lineEdit_laserResponse.setText("laser wavelength: " + wavelengthMaitai) 
        
    def setWavelength_maitai(self):
        wavelength_command = self.lineEdit_wavelength.text()
        print 'Setting laser wavelength... ' + wavelength_command +' nm'
        wavelengthMaitai=self.maitai.ChangeWavelength(int(wavelength_command))
        print wavelengthMaitai
        while (wavelengthMaitai[0:-3]!=wavelength_command):
            wavelengthMaitai = self.maitai.ReadWavelength()
        print 'wavelength set!'
        
    def setShutter2Ph(self,shutter):
        self.shutter2ph = shutter   
        
    def setShutter3Ph(self,shutter):
        self.shutter3ph = shutter       
            
    def toggle_shutter2ph(self):
        if self.shutter2ph_closed:
            self.shutter2ph.on()
            self.shutter2ph_closed = False
            self.pushButton_shutter_2ph.setStyleSheet("background-color: red");
            self.pushButton_shutter_2ph.setText('Shutter 2Ph: Open')
        else:
            self.shutter2ph.off() 
            self.shutter2ph_closed = True
            self.pushButton_shutter_2ph.setStyleSheet("background-color: pale gray");
            self.pushButton_shutter_2ph.setText('Shutter 2Ph: Closed')
        
    def toggle_shutter3ph(self):
        if self.shutter3ph_closed:
            self.shutter3ph.on()
            self.shutter3ph_closed = False
            self.pushButton_shutter_3ph.setStyleSheet("background-color: red");
            self.pushButton_shutter_3ph.setText('Shutter 3Ph: Open')
        else:
            self.shutter3ph.off() 
            self.shutter3ph_closed = True
            self.pushButton_shutter_3ph.setStyleSheet("background-color: pale gray");
            self.pushButton_shutter_3ph.setText('Shutter 3Ph: Closed')

    def setAoEOM(self,ao_eom):
        self.power_ao_eom = ao_eom
        
    def setPower2ph(self,value):
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        self.label_power2P.setText(str(int(value))+' %')
        eom_voltage=2.0*value/100.0
        self.power_ao_eom.write(eom_voltage)
    
    def setPower3ph(self,value):
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        if self.flagWheel == 1:
            self.label_power3P.setText(str(int(value))+' %')
            wheel3P_angle=self.wheel3PMax*value/100
            command = self.wheel3P_angle_offset+wheel3P_angle
            self.flagWheel = 0
            thread = threading.Thread(target=self.wheel_run, args=(command,))
            thread.start()
        
    def setPower3ph_noThread(self,value):
        if self.flagWheel == 1:
            self.label_power3P.setText(str(int(value))+' %')
            wheel3P_angle=self.wheel3PMax*value/100
            command = self.wheel3P_angle_offset+wheel3P_angle
            self.flagWheel = 0
            self.wheel_run(command)
   
        
    def setMinimumPower3ph(self):
        print('set minimum...')
        self.wheel3P_angle_offset=self.wheel3P_angle_offset+(360.0*self.horizontalScrollBar_power3ph.value()/100.0)
        print('half done...')
        self.horizontalScrollBar_power3ph.setValue(0)
        print('done!')
        
    def setMaximumPower3ph(self):
        print('set maximum...')        
        self.wheel3PMax=360.0*self.horizontalScrollBar_power3ph.value()/100
        self.horizontalScrollBar_power3ph.setValue(100)
        print(str(self.wheel3PMax))
        print('done!')   
        
    def resetWheel3ph(self):
        self.wheel3P_angle_offset=0
        self.wheel3PMax=360
        self.horizontalScrollBar_power3ph.setValue(0)

    def gotoMaximumPower3ph(self):
        print('going to maximum value with 3Ph motors...')
        wheel3P_angle=self.wheel3PMax
        command = self.wheel3P_angle_offset+wheel3P_angle
        self.flagWheel = 0
        thread = threading.Thread(target=self.wheel_run, args=(command,))
        thread.start()
        self.horizontalScrollBar_power3ph.setValue(100)
        print('done!')
        
    def gotoMaximumPower3ph_nothread(self):
        print('going to maximum value with 3Ph motors...')
        wheel3P_angle=self.wheel3PMax
        command = self.wheel3P_angle_offset+wheel3P_angle
        self.flagWheel = 0
        self.wheel_run(command)
        self.horizontalScrollBar_power3ph.setValue(100)
        print('done!')  

    def turnOffWheel3P(self):
        self.shutter3ph.off() 
        self.gotoMaximumPower3ph_nothread()

    def wheel_run(self,command):
        print "Starting rotation:"
        self.thorlabs.setpos(command)
        self.flagWheel = 1
        print "Wheel rotation done!"
        
#FUNCTIONS SETTING CLASSES:
        
    def setGalvos(self,galvos):
        self.galvos=galvos

    def setMotors(self,motors):
        self.motors=motors
        
    def setZaber(self,zaber):
        self.zaber=zaber
                
    def setThorlabs(self,thorlabs):
        self.thorlabs=thorlabs        
        
    def setPowerMotor(self,power_motor):
        self.power_motor = power_motor
        
    def setLaser(self,laser):
        self.laser = laser
    
    def setAiTask(self,ai_task):
        self.ai_task = ai_task
        
#FUNCTIONS POWER:
        
    def set_power1(self): 
        percent = float(self.lineEdit_laser_power1.text())
        #calib = 7*math.pi/45
        calib = 23*math.pi/45
        deg=float((45/math.pi)*(math.asin(2*percent/100-1)+math.pi/2+calib))
        print self.lineEdit_laser_power1.text()
        self.power_motor.goTo_abs(1, deg)
        
    def set_power2(self): 
        percent = float(self.lineEdit_laser_power2.text())
        #calib = 7*math.pi/45
        calib = 40*math.pi/45
        deg=float((45/math.pi)*(math.asin(2*percent/100-1)+math.pi/2+calib))
        print self.lineEdit_laser_power2.text()
        self.power_motor.goTo_abs(2, deg)
 
#FUNCTIONS MOTORS:
 
    def runReferenceX(self):
        #width=float(self.lineEdit_width.text())  
        #height=float(self.lineEdit_height.text())                  
        #self.start_snapshot(0,0,width,height)
        #time.sleep(5)
        self.refImageX=self.viewer.getCurrentImage()
        self.refImageX=self.refImageX[100:-100,100:-100]
        self.pushButtonMoveX.setEnabled(True)
        
    def runReferenceY(self):
        #height=float(self.lineEdit_height.text())                  
        #width=float(self.lineEdit_width.text())   
        #self.start_snapshot(0,0,width,height)
        #time.sleep(5)
        self.refImageY=self.viewer.getCurrentImage()
        self.refImageY=self.refImageY[100:-100,100:-100]
        self.pushButtonMoveY.setEnabled(True)
        
    def calibMoveX(self):
        width=float(self.lineEdit_width.text())      
        print('move in x...')
        self.cal_x_step=round(width/20)
        self.motors.move_dx(self.cal_x_step/1000)
        self.pushButton_calibrateX.setEnabled(True)
    
    def calibMoveY(self):
        height=float(self.lineEdit_height.text())                  
        print('move in x...')
        self.cal_y_step=round(height/20)
        self.motors.move_dy(self.cal_y_step/1000)
        self.pushButton_calibrateY.setEnabled(True)

    def runCalibrationX(self):
        height=float(self.lineEdit_height.text())                  
        width=float(self.lineEdit_width.text())      
        print('acquire image...')
        #self.start_snapshot(0,0,width,height)
        self.xShiftedImage=self.viewer.getCurrentImage()     
        self.xShiftedImage=self.xShiftedImage[100:-100,100:-100]   
        print('done!')
        self.runCalX=True
        if (self.runCalY and self.runCalX):
            self.pushButton_calibrateFOV.setEnabled(True)
 
    def runCalibrationY(self):
        height=float(self.lineEdit_height.text())                  
        width=float(self.lineEdit_width.text())         
        print('acquire image...')
        #self.start_snapshot(0,0,width,height)
        self.yShiftedImage=self.viewer.getCurrentImage() 
        self.yShiftedImage=self.yShiftedImage[100:-100,100:-100]
        print('done!')
        self.runCalY=True
        if (self.runCalY and self.runCalX):
            self.pushButton_calibrateFOV.setEnabled(True)

    def runFOVCalibration(self):

        #1. take reference image:
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text()) 
        nx=int(self.lineEdit_nx.text())       
        ny=int(self.lineEdit_ny.text())
               
        print('Cross-corr analysis:')
        print('X axis...')
        tempX=self.refImageX[(nx/2-75):(nx/2+75),(ny/2-75):(ny/2+75)]
        corrX = signal.correlate2d(self.xShiftedImage, tempX, boundary='symm', mode='same')
        y, x = np.unravel_index(np.argmax(corrX), corrX.shape) # find the match
        y=abs(y-ny/2)
        x=abs(x-nx/2)
        print(str(x)+' + '+str(y))
        if x>=y:
            shiftX=float(x)/float(nx)*width
        else:
            shiftX=float(y)/float(nx)*width
        print(str(shiftX))
        ratioX=self.cal_x_step/shiftX
        print('ratioX =' + str(ratioX))
        #y-shifting:
        print('Y axis...')
        tempY=self.refImageY[(nx/2-75):(nx/2+75),(ny/2-75):(ny/2+75)]
        corrY = signal.correlate2d(self.yShiftedImage, tempY, boundary='symm', mode='same')
        y, x = np.unravel_index(np.argmax(corrY), corrY.shape) # find the match
        y=abs(y-ny/2)
        x=abs(x-nx/2)
        
        print(str(x)+' + '+str(y))
        if x>=y:
            shiftY=float(x)/float(ny)*height
        else:
            shiftY=float(y)/float(ny)*height
        print(str(shiftY))    
        ratioY=self.cal_y_step/shiftY
        print('ratioY =' + str(ratioY))
        
        
        fig, ((ax_shiftx, ax_refx), (ax_shifty, ax_refy), (ax_corrx, ax_corry)) = plt.subplots(3, 2,figsize=(6, 15))
        ax_shiftx.imshow(self.xShiftedImage, cmap='gray')
        ax_shifty.imshow(self.yShiftedImage, cmap='gray')
        ax_refx.imshow(self.refImageX, cmap='gray')
        ax_refy.imshow(self.refImageY, cmap='gray')        
        ax_corrx.imshow(corrX, cmap='gray')
        ax_corry.imshow(corrY, cmap='gray')            
        fig.show()
        
    def set_brain_pos_reset(self):       
        print 'set_brain_pos: in function'
        self.brain_pos=self.motors.get_pos(3)
        print(self.brain_pos)
        print 'set_brain_pos: done'
        self.pushButton_goto_brain.setEnabled(True)    
        self.lineEdit_brain_pos.setText(self.brain_pos)
        self.brainPosSetFlag = 0
        
    def set_brain_pos(self):       
        print 'set_brain_pos: in function'
        self.brain_pos=self.motors.get_pos(3)
        print(self.brain_pos)
        print 'set_brain_pos: done'
        self.pushButton_goto_brain.setEnabled(True)    
        self.lineEdit_brain_pos.setText(self.brain_pos)
        self.brainPosSetFlag = 1
        self.pushButton_add_power_val.setEnabled(True)
        self.pushButton_add_power_val_3P.setEnabled(True)
        
    def get_current_z_depth(self):
        if self.brainPosSetFlag==1:
            self.abs_pos=self.motors.get_pos(3)
            self.currentZPos=float(self.brain_pos)-float(self.abs_pos)
            self.lineEdit_currentZpos.setText(str(self.currentZPos))
        else:
            print 'brain position not set yet!'
                
    def set_xy_center(self):       
        print 'set_x_center: in function'
        self.center_x=self.motors.get_pos(1)
        print(self.center_x)
        print 'set_x_center: done'
        print 'set_y_center: in function'
        self.center_y=self.motors.get_pos(2)
        print(self.center_y)
        print 'set_y_center: done'    
        self.pushButton_center.setEnabled(True)    
           
    def goto_brain_pos(self):       
        print 'goto_brain_pos: in function'
        #self.actual_pos=self.motors.get_pos_axial()
        #commandPos=float(self.brain_pos)-float(self.actual_pos)
        self.motors.move_az(self.brain_pos)
        self.get_current_z_depth()
        print 'goto_brain_pos: done'    

#FUNCTIONS SCANNING:
        
    def startlinescanTimer(self):
        self.currentLine = 0
        self.timerLS = pg.QtCore.QTimer()
        self.timerLS.timeout.connect(self.stoplinescanTimer)
        timeOffset=1.1 #offset of seconds measured   
        self.timestep=(float(self.lineEdit_TimePerLine.text())+timeOffset)*1000
        self.make_connection_offset_X()
        self.make_connection_offset_Y()
        self.make_connection_offset_Display()
        shift_display=self.horizontalScrollBar_line_scan_shift_display.value()
        self.viewer.move_offset_display(shift_display)
        self.viewer2.move_offset_display(shift_display)        
        self.runlinescanTimer()
        print('timestep of:' + str(int(self.timestep)))
        self.hideLines()
        
    def runlinescanTimer(self):
        
        self.viewer.setLineScanFlag(True)
        self.viewer2.setLineScanFlag(True)
        self.timerLS.start(self.timestep)        
        
        self.pushButtonStartMultipleLineAcq.setEnabled(False)
        self.pushButtonStopMultipleLineAcq.setEnabled(True)
        self.toggle_shutter2ph()
        self.get_current_z_depth()
        self.updateNumberOfLines()
        self.galvos.setFinite(False)
 
        x_0_px, y_0_px, x_e_px, y_e_px, linescan_length_px = self.viewer.getSelectedLinePosition(self.currentLine)
        
        nx_preview=int(self.lineEdit_nx.text())
        ny_preview=int(self.lineEdit_ny.text())
        width=float(self.lineEdit_width.text())
        height=float(self.lineEdit_height.text())
        
        self.linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value())
        x_0_px=x_0_px+self.linescan_shift_x
        x_e_px=x_e_px+self.linescan_shift_x
        
        self.linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value())
        y_0_px=y_0_px+self.linescan_shift_y
        y_e_px=y_e_px+self.linescan_shift_y
        
        x_0_um=(x_0_px/nx_preview*width)-width/2
        x_e_um=(x_e_px/nx_preview*width)-width/2
        y_0_um=(y_0_px/ny_preview*height)-height/2
        y_e_um=(y_e_px/ny_preview*height)-height/2 

        x_mean_px=(x_0_px+x_e_px)/2
        y_mean_px=(y_0_px+y_e_px)/2
        x_mean_um=(x_0_um+x_e_um)/2
        y_mean_um=(y_0_um+y_e_um)/2
        
        npts=int(self.lineEdit_nr.text())   
        n_lines=int(self.lineEdit_nt.text())   
        n_extra=int(self.lineEdit_extrapoints_LS.text())   
        line_rate=int(self.lineEdit_linerate_LS.text())   
        
        linescan_width_um = math.sqrt(math.pow((x_e_um-x_0_um),2)+math.pow((y_e_um-y_0_um),2))
        
        shift_display=self.horizontalScrollBar_line_scan_shift_display.value()
        #shift_display=self.linescan_shift_display
                
        self.galvos.setLineRamp(y_0_um,x_0_um,y_e_um,x_e_um,npts,n_lines,n_extra,line_rate,shift_display)
        self.galvos_stopped = False

        if self.checkBox_enable_save.isChecked():
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'LineScan'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.lineScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_linescan_acq.setText(str(self.lineScanNumber))
            self.scanNumber = self.scanType+'_'+str(self.lineScanNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            self.data_saver.setDatasetName(self.pathName)    
            self.data_saver.addAttribute('npts',npts)
            self.data_saver.addAttribute('n_lines',n_lines)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.addAttribute('x_0_px',x_0_px)            
            self.data_saver.addAttribute('x_e_px',x_e_px)              
            self.data_saver.addAttribute('y_0_px',y_0_px)            
            self.data_saver.addAttribute('y_e_px',y_e_px)                
            self.data_saver.addAttribute('x_mean_px',x_mean_px)            
            self.data_saver.addAttribute('y_mean_px',y_mean_px) 
            self.data_saver.addAttribute('x_0_um',x_0_um)            
            self.data_saver.addAttribute('x_e_um',x_e_um)              
            self.data_saver.addAttribute('y_0_um',y_0_um)            
            self.data_saver.addAttribute('y_e_um',y_e_um)                
            self.data_saver.addAttribute('x_mean_um',x_mean_um)            
            self.data_saver.addAttribute('y_mean_um',y_mean_um) 
            self.data_saver.addAttribute('depth',self.currentZPos)
            self.data_saver.setBlockSize(512)          
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            self.data_saver.startSaving()
        self.updateLineAcqTimer()
        self.galvos.startTask()        

    def updateLineAcqTimer(self):
        txt='Acquiring Line '+str(self.currentLine+1)+' out of '+str(self.numberOfLines)
        self.label_lineScanAcq.setText(txt)

    def abortlinescanTimer(self):
        print('Aborting linescan')
        self.currentLine=self.numberOfLines-1
        self.stoplinescanTimer()
            
       
    def stoplinescanTimer(self):
        #check if this is last line or not:
        self.timerLS.stop()
        self.toggle_shutter2ph()
        self.galvos.stopTask()    
        self.galvos_stopped = True     
        if self.checkBox_enable_save.isChecked():
            self.data_saver.stopSaving()
            self.ai_task.removeDataConsumer(self.data_saver)

        print('finished line: ' + str(self.currentLine+1) + ' out of ' + str(self.numberOfLines))
            
        if (self.numberOfLines==self.currentLine+1):
            print('finished all acquisitions!')
            self.viewer.displayLines()
            self.pushButton_start_linescan.setEnabled(True)
            self.pushButton_stop_linescan.setEnabled(False)
            self.pushButton_get_line_position.setEnabled(True)
            self.pushButton_start.setEnabled(True)      
            self.currentLine=0
            self.viewer.setLineScanFlag(False)
            self.viewer2.setLineScanFlag(False)
        else:
            time.sleep(2)
            self.currentLine = self.currentLine + 1
            self.runlinescanTimer()
            print('running next acquisition:...')

            
        
    def startlinescan(self):
        x_0_px, y_0_px, x_e_px, y_e_px, linescan_length_px = self.viewer.getMouseSelectedLinePosition()
        self.viewer.setLineScanFlag(True)
        self.viewer2.setLineScanFlag(True)
        if (x_0_px==0 and y_0_px==0 and x_e_px==0 and y_e_px==0):
            print("no lines selected!")
        else:
            self.toggle_shutter2ph()
            self.viewer.toggleLinearSelection()
            self.make_connection_offset_X()
            self.make_connection_offset_Y()
            self.make_connection_offset_Display()
            self.get_current_z_depth()
        
            if self.previewScanFlag == True:
                self.stopscan()
                self.toggle_shutter2ph()
        
            self.pushButton_start_linescan.setEnabled(False)
            self.pushButton_stop_linescan.setEnabled(True)
            self.pushButton_get_line_position.setEnabled(False)
            
            nx_preview=int(self.lineEdit_nx.text())
            ny_preview=int(self.lineEdit_ny.text())
            width=float(self.lineEdit_width.text())
            height=float(self.lineEdit_height.text())
            
            self.linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value())
            x_0_px=x_0_px+self.linescan_shift_x
            x_e_px=x_e_px+self.linescan_shift_x
            
            self.linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value())
            y_0_px=y_0_px+self.linescan_shift_y
            y_e_px=y_e_px+self.linescan_shift_y
            
            x_0_um=(x_0_px/nx_preview*width)-width/2
            x_e_um=(x_e_px/nx_preview*width)-width/2
            y_0_um=(y_0_px/ny_preview*height)-height/2
            y_e_um=(y_e_px/ny_preview*height)-height/2 
    
            x_mean_px=(x_0_px+x_e_px)/2
            y_mean_px=(y_0_px+y_e_px)/2
            x_mean_um=(x_0_um+x_e_um)/2
            y_mean_um=(y_0_um+y_e_um)/2
            
            npts=int(self.lineEdit_nr.text())   
            n_lines=int(self.lineEdit_nt.text())   
            n_extra=int(self.lineEdit_extrapoints_LS.text())   
            line_rate=int(self.lineEdit_linerate_LS.text())   
            
            linescan_width_um = math.sqrt(math.pow((x_e_um-x_0_um),2)+math.pow((y_e_um-y_0_um),2))
            
            shift_display=self.horizontalScrollBar_line_scan_shift_display.value()
            self.viewer.move_offset_display(shift_display)
            self.viewer2.move_offset_display(shift_display)
            
            self.galvos.setLineRamp(y_0_um,x_0_um,y_e_um,x_e_um,npts,n_lines,n_extra,line_rate,shift_display)
            self.galvos_stopped = False
            self.pushButton_start.setEnabled(False)
    
            if self.checkBox_enable_save.isChecked():
                self.data_saver=DataSaver(self.save_filename)
                self.mouseName=self.lineEdit_mouse_name.text()
                self.scanType = 'LineScan'      
                self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
                
                self.lineScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
                self.lineEdit_linescan_acq.setText(str(self.lineScanNumber))
                self.scanNumber = self.scanType+'_'+str(self.lineScanNumber) 
                self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
                self.data_saver.setDatasetName(self.pathName)    
                self.data_saver.addAttribute('npts',npts)
                self.data_saver.addAttribute('n_lines',n_lines)
                self.data_saver.addAttribute('n_extra',n_extra)
                self.data_saver.addAttribute('width',width)
                self.data_saver.addAttribute('height',height)            
                self.data_saver.addAttribute('line_rate',line_rate)   
                self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
                self.data_saver.addAttribute('x_0_px',x_0_px)            
                self.data_saver.addAttribute('x_e_px',x_e_px)              
                self.data_saver.addAttribute('y_0_px',y_0_px)            
                self.data_saver.addAttribute('y_e_px',y_e_px)                
                self.data_saver.addAttribute('x_mean_px',x_mean_px)            
                self.data_saver.addAttribute('y_mean_px',y_mean_px) 
                self.data_saver.addAttribute('x_0_um',x_0_um)            
                self.data_saver.addAttribute('x_e_um',x_e_um)              
                self.data_saver.addAttribute('y_0_um',y_0_um)            
                self.data_saver.addAttribute('y_e_um',y_e_um)                
                self.data_saver.addAttribute('x_mean_um',x_mean_um)            
                self.data_saver.addAttribute('y_mean_um',y_mean_um) 
                self.data_saver.addAttribute('depth',self.currentZPos)
                self.data_saver.setBlockSize(512)          
                self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
                self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
                self.data_saver.startSaving()
            self.galvos.startTask()


    def stoplinescan(self):
        self.toggle_shutter2ph()
        self.viewer.toggleLinearSelection()
        
        self.pushButton_start_linescan.setEnabled(True)
        self.pushButton_stop_linescan.setEnabled(False)
        self.pushButton_get_line_position.setEnabled(True)
        if self.checkBox_enable_save.isChecked():
            self.data_saver.stopSaving()
            self.ai_task.removeDataConsumer(self.data_saver)
        self.galvos.stopTask()    
        self.galvos_stopped = True
        self.pushButton_start.setEnabled(True)
        self.viewer.setLineScanFlag(False)
        self.viewer2.setLineScanFlag(False)
        
    def startscan(self):
        #self.liveScanNumber=self.liveScanNumber+1;
        self.previewScanFlag = True
        self.pushButton_stop.setEnabled(True)
        self.pushButton_start.setEnabled(False)

        self.shutter2ph_closed=self.checkBoxLive2P.isChecked()
        self.shutter3ph_closed=self.checkBoxLive3P.isChecked()
        self.toggle_shutter2ph()
        self.toggle_shutter3ph()
        self.checkBoxLive2P.setEnabled(False)
        self.checkBoxLive3P.setEnabled(False)
        
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text())                     
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())         
        line_rate=float(self.lineEdit_linerate.text())
        
        
        # Set ramp
        if self.comboBox_scantype.currentText() == 'SawTooth':
            self.galvos.setSawToothRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
        elif self.comboBox_scantype.currentText() == 'Triangular':
            self.galvos.setTriangularRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
        elif self.comboBox_scantype.currentText() == 'Line':          
            self.galvos.setLineRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,line_rate)
        # Start generating
        self.galvos_stopped = False
        
        if self.checkBox_enable_save.isChecked():
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'LiveScan'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.liveScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_live_acq.setText(str(self.liveScanNumber))
            self.scanNumber = self.scanType+'_'+str(self.liveScanNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            self.data_saver.setDatasetName(self.pathName)       
            self.data_saver.addAttribute('nx',nx)
            self.data_saver.addAttribute('ny',ny)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.setBlockSize(512)          
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            self.data_saver.startSaving()
        print('scanning started')            
        self.galvos.startTask()

    def stopscan(self):
        print('stop scanning')
        self.shutter2ph_closed=False
        self.shutter3ph_closed=False  
        self.toggle_shutter2ph()
        self.toggle_shutter3ph()
        if self.checkBox_enable_save.isChecked():
            print('stop saving...')
            self.data_saver.stopSaving()
            self.ai_task.removeDataConsumer(self.data_saver)
            #self.data_saver.setBlockSize(512)
            print('stop saving done!')
        self.galvos.stopTask()    
        self.galvos_stopped = True
        print('stop scanning done!')
        
        self.pushButton_stop.setEnabled(False)
        self.pushButton_start.setEnabled(True)
        
        self.checkBoxLive2P.setEnabled(True)
        self.checkBoxLive3P.setEnabled(True)        
        self.previewScanFlag = False

    def stack_thread(self):
        self.galvos.setFinite(True)
        self.goto_brain_pos()
        self.progressBar_stack.setValue(0)
        
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text())                     
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())         
        line_rate=float(self.lineEdit_linerate.text())
        self.progressBar_stack.setValue(50)        
        n=0         
        time.sleep(1)
        # Step two: Start stack loop
        self.zstep=float(self.lineEdit_zstep.text())
        n_steps = int(self.lineEdit_nstack.text())
        
        for k in range(n_steps):
            # Take 2D image and saving
            # Set ramp
            if self.comboBox_scantype.currentText() == 'SawTooth':
                self.galvos.setSawToothRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
            elif self.comboBox_scantype.currentText() == 'Triangular':
                self.galvos.setTriangularRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
            elif self.comboBox_scantype.currentText() == 'Line':          
                self.galvos.setLineRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,line_rate)

            self.galvos_stopped = False
            self.progressBar_stack.setValue(k/n_steps*100)
            print k
                       
            print('scanning started')   
            self.make_connection(self.galvos.ao_task.signal_helper.aoDoneSignal)         
            self.galvos.startTask()    
        
            print 'scan started'
            self.motors.move_dz(-self.zstep/1000)
            
            print 'motor moved'
            time.sleep(1)
            if self.stopped:
                break
        self.goto_brain_pos()

        #time.sleep(2)
        
        self.galvos.setFinite(False)
    
    def take_snapshot(self):
        
        x_0_px, y_0_px, x_e_px, y_e_px, linescan_length_px = self.viewer.getCurrentLinePosition()
        nx_preview=int(self.lineEdit_nx.text())
        ny_preview=int(self.lineEdit_ny.text())
        width=float(self.lineEdit_width.text())
        height=float(self.lineEdit_height.text())
        
        self.linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value())
        x_0_px=x_0_px+self.linescan_shift_x
        x_e_px=x_e_px+self.linescan_shift_x
        
        self.linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value())
        y_0_px=y_0_px+self.linescan_shift_y
        y_e_px=y_e_px+self.linescan_shift_y
        
        x_0_um=(x_0_px/nx_preview*width)-width/2
        x_e_um=(x_e_px/nx_preview*width)-width/2
        y_0_um=(y_0_px/ny_preview*height)-height/2
        y_e_um=(y_e_px/ny_preview*height)-height/2 
        
        x_mean_px=(x_0_px+x_e_px)/2
        y_mean_px=(y_0_px+y_e_px)/2
               
        x_mean_um=(x_0_um+x_e_um)/2
        y_mean_um=(y_0_um+y_e_um)/2
        
        center_x=x_mean_um;
        center_y=y_mean_um;
        width=50          
        height=50
        self.start_snapshot(center_y,center_x,height,width)
    
    def start_snapshot(self,center_x,center_y,width,height):
        #self.stackNumber=self.stackNumber+1;
        self.pushButton_start_stack.setEnabled(False)
        self.pushButton_stop_stack.setEnabled(True)
        self.get_current_z_depth()
        self.shutter2ph_closed=self.checkBoxLive2P.isChecked()
        self.shutter3ph_closed=self.checkBoxLive3P.isChecked()
        
        print('shutter 2p:')
        print(self.shutter2ph_closed)
        
        self.toggle_shutter2ph()
        self.toggle_shutter3ph()
        self.checkBoxStack2P.setEnabled(False)
        self.checkBoxStack3P.setEnabled(False)
        self.galvos.setFinite(True)
        
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text())                             
        line_rate=float(self.lineEdit_linerate.text())
                
        self.galvos.setSawToothRamp(center_x-width/2,center_y-height/2,center_x+width/2,center_y+height/2,nx,ny,n_extra,1,line_rate)
        self.galvos_stopped = False
        #SET SAVING PART:
        if self.checkBox_enable_save.isChecked():
            print 'setting saving'
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'snapshot'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.stackNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            print 'data checked'
            self.lineScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_linescan_acq.setText(str(self.lineScanNumber))
            self.scanNumber = self.scanType+'_'+str(self.lineScanNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            print 'setting name:'
            print self.pathName            
            self.data_saver.setDatasetName(self.pathName)       
            self.data_saver.addAttribute('nx',nx)
            self.data_saver.addAttribute('ny',ny)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.setBlockSize(512)
            print 'setting consumers'
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            print 'starting saving'
            self.data_saver.startSaving()
        #        print 'making connection'
        self.make_connection_snapshot(self.galvos.ao_task.signal_helper.aoDoneSignal)
        print 'starting task'       
        self.galvos.startTask()
    
    @pyqtSlot()
    def stop_snapshot(self):
        self.galvos_stopped=True        
        print('acquisition finished!')
        self.shutter2ph_closed=False
        self.shutter3ph_closed=False  
        self.toggle_shutter2ph()
        self.toggle_shutter3ph()
        self.checkBoxStack2P.setEnabled(True)
        self.checkBoxStack3P.setEnabled(True)   
        self.galvos.setFinite(False)
        self.progressBar_stack.setValue(0)
        self.pushButton_start_stack.setEnabled(True)
        self.pushButton_stop_stack.setEnabled(False)
        if self.checkBox_enable_save.isChecked():
            print('stop saving...')
            self.data_saver.stopSaving()
            self.ai_task.removeDataConsumer(self.data_saver)
                #self.data_saver.setBlockSize(512)
            print('stop saving done!')
            
            
    def make_connection_snapshot(self, slider_object):
        slider_object.connect(self.stop_snapshot)
    
    #MULTIPLE LINE SCANS
    
    def start_multiple_lines(self):
        self.viewer.setLineScanFlag(True)
        self.viewer2.setLineScanFlag(True)
        self.pushButtonStartMultipleLineAcq.setEnabled(False)
        self.pushButtonStopMultipleLineAcq.setEnabled(True)
        print('in START MULTIPLE LINES')  
        self.toggle_shutter2ph()
        self.make_connection_offset_X()
        self.make_connection_offset_Y()
        self.make_connection_offset_Display()
        self.get_current_z_depth()
        self.updateNumberOfLines()
        self.galvos.setFinite(True)
        self.hideLines()
        self.lineRepeat=int(self.lineEdit_lineRepeat.text())
                
        self.currentLine = 0
        self.currentIteration = 0
        x_0_px, y_0_px, x_e_px, y_e_px, linescan_length_px = self.viewer.getSelectedLinePosition(self.currentLine)

        nx_preview=int(self.lineEdit_nx.text())
        ny_preview=int(self.lineEdit_ny.text())
        width=float(self.lineEdit_width.text())
        height=float(self.lineEdit_height.text())
        
        self.linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value())
        x_0_px=x_0_px+self.linescan_shift_x
        x_e_px=x_e_px+self.linescan_shift_x
        
        self.linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value())
        y_0_px=y_0_px+self.linescan_shift_y
        y_e_px=y_e_px+self.linescan_shift_y
        
        x_0_um=(x_0_px/nx_preview*width)-width/2
        x_e_um=(x_e_px/nx_preview*width)-width/2
        y_0_um=(y_0_px/ny_preview*height)-height/2
        y_e_um=(y_e_px/ny_preview*height)-height/2 

        x_mean_px=(x_0_px+x_e_px)/2
        y_mean_px=(y_0_px+y_e_px)/2
        x_mean_um=(x_0_um+x_e_um)/2
        y_mean_um=(y_0_um+y_e_um)/2
        
        npts=int(self.lineEdit_nr.text())   
        n_lines=int(self.lineEdit_nt.text())   
        n_extra=int(self.lineEdit_extrapoints_LS.text())   
        line_rate=int(self.lineEdit_linerate_LS.text())   
        
        linescan_width_um = math.sqrt(math.pow((x_e_um-x_0_um),2)+math.pow((y_e_um-y_0_um),2))
        
        shift_display=self.horizontalScrollBar_line_scan_shift_display.value()
        #shift_display=self.linescan_shift_display
                
        self.galvos.setLineRamp(y_0_um,x_0_um,y_e_um,x_e_um,npts,n_lines,n_extra,line_rate,shift_display)
        self.galvos_stopped = False

        if self.checkBox_enable_save.isChecked():
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'LineScan'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.lineScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_linescan_acq.setText(str(self.lineScanNumber))
            self.scanNumber = self.scanType+'_'+str(self.lineScanNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            self.data_saver.setDatasetName(self.pathName)    
            self.data_saver.addAttribute('npts',npts)
            self.data_saver.addAttribute('n_lines',n_lines)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.addAttribute('x_0_px',x_0_px)            
            self.data_saver.addAttribute('x_e_px',x_e_px)              
            self.data_saver.addAttribute('y_0_px',y_0_px)            
            self.data_saver.addAttribute('y_e_px',y_e_px)                
            self.data_saver.addAttribute('x_mean_px',x_mean_px)            
            self.data_saver.addAttribute('y_mean_px',y_mean_px) 
            self.data_saver.addAttribute('x_0_um',x_0_um)            
            self.data_saver.addAttribute('x_e_um',x_e_um)              
            self.data_saver.addAttribute('y_0_um',y_0_um)            
            self.data_saver.addAttribute('y_e_um',y_e_um)                
            self.data_saver.addAttribute('x_mean_um',x_mean_um)            
            self.data_saver.addAttribute('y_mean_um',y_mean_um) 
            self.data_saver.addAttribute('depth',self.currentZPos)
            self.data_saver.setBlockSize(512)          
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            self.data_saver.startSaving()
        self.make_connection_multiple_lines(self.galvos.ao_task.signal_helper.aoDoneSignal)
        self.updateLineAcq()
        self.galvos.startTask()        
    
    def updateLineAcq(self):
        currentFrame = (self.currentIteration % self.lineRepeat) + 1
        txt='Acquiring frame '+str(currentFrame)+' out of '+str(self.lineRepeat) + ' for Line '+str(self.currentLine+1)+' out of '+str(self.numberOfLines)
        self.label_lineScanAcq.setText(txt)
    
    def stop_multiple_lines(self):
        print('stop pushed...')
        self.galvos_stopped=True
        print('...done!')
        self.viewer.setLineScanFlag(False)
        self.viewer2.setLineScanFlag(False)

    def make_connection_multiple_lines(self,slider_object):
        print('making connection multiple lines...')
        slider_object.connect(self.nextLine)
        print('...done!')

    @pyqtSlot()       
    def nextLine(self):
        print('in NEXTLINE')
        if self.checkBox_enable_save.isChecked():
            self.data_saver.stopSaving()
            self.ai_task.removeDataConsumer(self.data_saver)
        time.sleep(2)
        self.currentIteration = self.currentIteration+1;        
        testAveraging = self.currentIteration % self.lineRepeat       
        if (testAveraging==0) :
            self.currentLine = self.currentLine + 1
            
        galvo_on=not(self.galvos_stopped)
        iterationCond=((self.numberOfLines-1)>=self.currentLine)
        
        if (iterationCond & galvo_on):
            self.galvos_stopped = False
            x_0_px, y_0_px, x_e_px, y_e_px, linescan_length_px = self.viewer.getSelectedLinePosition(self.currentLine)

            nx_preview=int(self.lineEdit_nx.text())
            ny_preview=int(self.lineEdit_ny.text())
            width=float(self.lineEdit_width.text())
            height=float(self.lineEdit_height.text())
        
            self.linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value())
            x_0_px=x_0_px+self.linescan_shift_x
            x_e_px=x_e_px+self.linescan_shift_x
        
            self.linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value())
            y_0_px=y_0_px+self.linescan_shift_y
            y_e_px=y_e_px+self.linescan_shift_y
        
            x_0_um=(x_0_px/nx_preview*width)-width/2
            x_e_um=(x_e_px/nx_preview*width)-width/2
            y_0_um=(y_0_px/ny_preview*height)-height/2
            y_e_um=(y_e_px/ny_preview*height)-height/2 

            x_mean_px=(x_0_px+x_e_px)/2
            y_mean_px=(y_0_px+y_e_px)/2
            x_mean_um=(x_0_um+x_e_um)/2
            y_mean_um=(y_0_um+y_e_um)/2
        
            npts=int(self.lineEdit_nr.text())   
            n_lines=int(self.lineEdit_nt.text())   
            n_extra=int(self.lineEdit_extrapoints_LS.text())   
            line_rate=int(self.lineEdit_linerate_LS.text())   
        
            linescan_width_um = math.sqrt(math.pow((x_e_um-x_0_um),2)+math.pow((y_e_um-y_0_um),2))
        
            shift_display=self.linescan_shift_display
                
            self.galvos.setLineRamp(y_0_um,x_0_um,y_e_um,x_e_um,npts,n_lines,n_extra,line_rate,shift_display)
            
            if (testAveraging == 0):

                if self.checkBox_enable_save.isChecked():
                    self.data_saver=DataSaver(self.save_filename)
                    self.mouseName=self.lineEdit_mouse_name.text()
                    self.scanType = 'LineScan'      
                    self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
                
                    self.lineScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
                    self.lineEdit_linescan_acq.setText(str(self.lineScanNumber))
                    self.scanNumber = self.scanType+'_'+str(self.lineScanNumber) 
                    self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
                    self.data_saver.setDatasetName(self.pathName)    
                    self.data_saver.addAttribute('npts',npts)
                    self.data_saver.addAttribute('n_lines',n_lines)
                    self.data_saver.addAttribute('n_extra',n_extra)
                    self.data_saver.addAttribute('width',width)
                    self.data_saver.addAttribute('height',height)            
                    self.data_saver.addAttribute('line_rate',line_rate)   
                    self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
                    self.data_saver.addAttribute('x_0_px',x_0_px)            
                    self.data_saver.addAttribute('x_e_px',x_e_px)              
                    self.data_saver.addAttribute('y_0_px',y_0_px)            
                    self.data_saver.addAttribute('y_e_px',y_e_px)                
                    self.data_saver.addAttribute('x_mean_px',x_mean_px)            
                    self.data_saver.addAttribute('y_mean_px',y_mean_px) 
                    self.data_saver.addAttribute('x_0_um',x_0_um)            
                    self.data_saver.addAttribute('x_e_um',x_e_um)              
                    self.data_saver.addAttribute('y_0_um',y_0_um)            
                    self.data_saver.addAttribute('y_e_um',y_e_um)                
                    self.data_saver.addAttribute('x_mean_um',x_mean_um)            
                    self.data_saver.addAttribute('y_mean_um',y_mean_um) 
                    self.data_saver.addAttribute('depth',self.currentZPos)
                    self.data_saver.setBlockSize(512)          
                    self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
                    self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
                    self.data_saver.startSaving()
                    
            self.make_connection_multiple_lines(self.galvos.ao_task.signal_helper.aoDoneSignal)
            self.updateLineAcq()
            self.galvos.startTask()
        else:
            print('acquisition finished!')
            self.galvos.setFinite(False)
            self.displayLines()
            self.pushButtonStartMultipleLineAcq.setEnabled(True)
            self.pushButtonStopMultipleLineAcq.setEnabled(False)
            self.toggle_shutter2ph()
            if self.checkBox_enable_save.isChecked():
                self.data_saver.stopSaving()
                self.ai_task.removeDataConsumer(self.data_saver)

    #STACK
    
    def start_stack_thread(self):
        if self.checkBoxPowerCurve.isChecked():
            self.generatePowerCurve()
        if self.checkBoxPowerCurve3P.isChecked():
            self.generatePowerCurve3P()
        #self.stackNumber=self.stackNumber+1;
        self.pushButton_start_stack.setEnabled(False)
        self.pushButton_stop_stack.setEnabled(True)
        
        self.shutter2ph_closed=self.checkBoxStack2P.isChecked()
        self.shutter3ph_closed=self.checkBoxStack3P.isChecked()
        self.toggle_shutter2ph()
        self.toggle_shutter3ph()
        self.checkBoxStack2P.setEnabled(False)
        self.checkBoxStack3P.setEnabled(False)
        
        self.galvos.setFinite(True)
        self.goto_brain_pos()
        self.progressBar_stack.setValue(0)
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text())                     
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())         
        line_rate=float(self.lineEdit_linerate.text())
        self.progressBar_stack.setValue(1)        
        time.sleep(1)
        # Step two: Start stack loop
        self.zstep=float(self.lineEdit_zstep.text())
        self.n_steps = int(self.lineEdit_nstack.text())
        # Take 2D image and saving
        # Set ramp
        self.powerIt=0;
        if self.comboBox_scantype.currentText() == 'SawTooth':
            self.galvos.setSawToothRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
        elif self.comboBox_scantype.currentText() == 'Triangular':
            self.galvos.setTriangularRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
        elif self.comboBox_scantype.currentText() == 'Line':          
            self.galvos.setLineRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,line_rate)
        self.galvos_stopped = False
        #SET SAVING PART:
        if self.checkBox_enable_save.isChecked():
            print 'setting saving'
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'Stack'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.stackNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            print 'data checked'
            self.lineEdit_stack_acq.setText(str(self.stackNumber))
            self.scanNumber = self.scanType+'_'+str(self.stackNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            print 'setting name:'
            print self.pathName            
            self.data_saver.setDatasetName(self.pathName)       
            self.data_saver.addAttribute('nx',nx)
            self.data_saver.addAttribute('ny',ny)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.addAttribute('averaging',int(self.lineEdit_averaging.text()))  
            self.data_saver.addAttribute('n_steps',self.n_steps)  
            self.data_saver.addAttribute('zstep',self.zstep)              
            self.data_saver.setBlockSize(512)
            print 'setting consumers'
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            print 'starting saving'
            self.get_current_z_depth()
            self.lineEdit_current_depth.setText(str(self.currentZPos))
            self.lineEdit_current_depth_3P.setText(str(self.currentZPos))

            self.data_saver.startSaving()
        #
        self.set_iteration_number(0)
        print 'making connection'
        self.make_connection(self.galvos.ao_task.signal_helper.aoDoneSignal)
        
        if self.checkBoxPowerCurve.isChecked():
            currentPowerValue=self.powerCurve[0]
            self.setPower2ph(currentPowerValue)
            self.horizontalScrollBar_power2ph.setValue(currentPowerValue)
            txt= str(round(currentPowerValue,1)) + '%'
            self.lineEdit_current_power.setText(txt)

        if self.checkBoxPowerCurve3P.isChecked():
            currentPowerValue=self.powerCurve3P[0]
            self.setPower3ph_noThread(currentPowerValue)
            self.horizontalScrollBar_power3ph.setValue(currentPowerValue)
            txt= str(round(currentPowerValue,1)) + '%'
            self.lineEdit_current_power_3P.setText(txt)
        
        print 'starting task'                    
        self.galvos.startTask()    
    
    def stop_stack(self):
        self.galvos_stopped=True        

    def make_connection(self, slider_object):
        slider_object.connect(self.nextAcq)
        
    @pyqtSlot()
    def nextAcq(self):
        print('in nextAcq')
        self.iteration_number=self.get_iteration_number()
        progVal=self.iteration_number*100/self.n_steps+1
        self.progressBar_stack.setValue(progVal)        
        self.galvos.stopTask()    
        self.set_iteration_number(self.iteration_number+1) 
        self.averagingVal = int(self.lineEdit_averaging.text())
         
        self.lineEditSliceUnderAcq.setText(str(self.iteration_number/self.averagingVal))  
        self.lineEdit_AvgNumber.setText(str(((self.iteration_number) % self.averagingVal)+1))
        
        stepVal=(self.n_steps*self.averagingVal)-1
        iterationCond=(self.iteration_number < stepVal)
        galvo_on=not(self.galvos_stopped)
        print(self.iteration_number)
        if (iterationCond & galvo_on):
            if ((((self.iteration_number) % self.averagingVal)==0) & (self.iteration_number>0)):
                print('moving motors')
                self.motors.move_dz(-self.zstep/1000)
                print('done!')
                self.powerIt=self.powerIt+1;
                print('power number: ' + str(self.powerIt))
                if self.checkBoxPowerCurve.isChecked():
                    currentPowerValue=self.powerCurve[self.powerIt]
                    print('2P: power value: ' + str(round(currentPowerValue,1)) + '%')
                    if self.checkBoxStack2P.isChecked():
                        self.setPower2ph(currentPowerValue)
                        self.horizontalScrollBar_power2ph.setValue(currentPowerValue)
                        txt= str(round(currentPowerValue,1)) + '%'
                        self.lineEdit_current_power.setText(txt)
                if self.checkBoxPowerCurve3P.isChecked():
                    currentPowerValue3P=self.powerCurve3P[self.powerIt]
                    print('3P: power value: ' + str(round(currentPowerValue3P,1)) + '%')
                    if self.checkBoxStack3P.isChecked():                    
                        self.setPower3ph_noThread(currentPowerValue3P)
                        self.horizontalScrollBar_power3ph.setValue(currentPowerValue3P)
                        txt= str(round(currentPowerValue3P,1)) + '%'
                        self.lineEdit_current_power_3P.setText(txt)
            time.sleep(2)
            nx=int(self.lineEdit_nx.text())    
            ny=int(self.lineEdit_ny.text())   
            n_extra=int(self.lineEdit_extrapoints.text())                     
            width=float(self.lineEdit_width.text())          
            height=float(self.lineEdit_height.text())         
            line_rate=float(self.lineEdit_linerate.text())
            if self.comboBox_scantype.currentText() == 'SawTooth':
                self.galvos.setSawToothRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
            elif self.comboBox_scantype.currentText() == 'Triangular':
                self.galvos.setTriangularRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
            elif self.comboBox_scantype.currentText() == 'Line':          
                self.galvos.setLineRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,line_rate)    
            self.get_current_z_depth()
            self.lineEdit_current_depth.setText(str(self.currentZPos))
            self.lineEdit_current_depth_3P.setText(str(self.currentZPos))
            self.make_connection(self.galvos.ao_task.signal_helper.aoDoneSignal)                
            self.galvos.startTask()
        else:
            print('acquisition finished!')
            self.shutter2ph_closed=False
            self.shutter3ph_closed=False  
            self.toggle_shutter2ph()
            self.toggle_shutter3ph()
            self.checkBoxStack2P.setEnabled(True)
            self.checkBoxStack3P.setEnabled(True)   
            self.goto_brain_pos()
            self.get_current_z_depth()
            self.galvos.setFinite(False)
            self.progressBar_stack.setValue(0)
            self.pushButton_start_stack.setEnabled(True)
            self.pushButton_stop_stack.setEnabled(False)
            if self.checkBoxPowerCurve.isChecked():
                self.setPower2ph(0)
                self.horizontalScrollBar_power2ph.setValue(0)
            if self.checkBoxPowerCurve3P.isChecked():
                self.setPower3ph(0)
                self.horizontalScrollBar_power3ph.setValue(0)
            if self.checkBox_enable_save.isChecked():
                print('stop saving...')
                self.data_saver.stopSaving()
                self.ai_task.removeDataConsumer(self.data_saver)
                #self.data_saver.setBlockSize(512)
                print('stop saving done!')
        #time.sleep(2)
        
    def make_connection_offset_X(self):
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.galvos.move_offset_X)        

    def make_connection_offset_Y(self):
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.galvos.move_offset_Y)        
        
    def make_connection_offset_Display(self):
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.viewer.move_offset_display)        
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.viewer2.move_offset_display)        
    
    def set_iteration_number(self,val):
        self.iterationNumber=val
        
    def get_iteration_number(self):
        return self.iterationNumber

    def scantype_chosen(self,text):
        self.galvos.stopTask()
        if text=='Sawtooth':
            self.label_nx.setText('Nx')
            self.label_ny.setText('Ny')
        elif text=='Triangular':
            self.label_nx.setText('Nx')
            self.label_ny.setText('Ny')            
        elif text=='Line':
            self.label_nx.setText('N points')
            self.label_ny.setText('N repeat')

# Motors for XYZ movement            
    def move_up(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())
        self.motors.move_dy(delta_xy/1000)
    def move_down(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())     
        self.motors.move_dy(-delta_xy/1000)
    def move_right(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())     
        self.motors.move_dx(delta_xy/1000)
    def move_left(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())     
        self.motors.move_dx(-delta_xy/1000)
    def move_center(self):
        print 'goto_x_center: in function'
        self.motors.move_ax(self.center_x)
        print 'goto_x_center: done'
        print 'goto_y_center: in function'
        self.motors.move_ay(self.center_y)
        print 'goto_y_center: done'    

    def move_z_up(self):       
        delta_z=float(self.lineEdit_z_motor_step.text())
        self.motors.move_dz(delta_z/1000)
        print 'move done'
        self.get_current_z_depth()
        
    def go_to_pos_axial(self):
        if self.brainPosSetFlag==1:
            self.abs_pos=self.motors.get_pos(3)
            self.depth_command=float(self.lineEdit_DepthCommand.text())/1000
            self.depth_command=float(self.brain_pos)-self.depth_command
            self.motors.move_dz(self.depth_command)
        else:
            print('brain position not set yet!')
        
    def move_z_down(self):
        delta_z=float(self.lineEdit_z_motor_step.text())
        self.motors.move_dz(-delta_z/1000)   
        self.get_current_z_depth()
     
    def move_motor_brain_surface(self,brain_pos):
        self.motors.move_ax(brain_pos)
        
    def move_ZaberX_left(self):
        posX=[-float(self.lineEdit_Zaber_step.text())*10,0]
        self.zaber.goTo(posX)

    def move_ZaberX_right(self):
        posX=[float(self.lineEdit_Zaber_step.text())*10,0]
        self.zaber.goTo(posX)
        
    def move_ZaberY_left(self):
        posY=[0,-float(self.lineEdit_Zaber_step.text())*10]
        self.zaber.goTo(posY)        
        
    def move_ZaberY_right(self):
        posY=[0,float(self.lineEdit_Zaber_step.text())*10]
        self.zaber.goTo(posY)
        
    def initialisation(self):
        print 'going home'
        self.thorlabs.goHome()
        self.zaber.goHome()
        self.motors.home()
        time.sleep(3)
        self.motors.move_ax(-11.6)
        
    def exitprogram(self):
        self.thorlabs.cleanUpAPT()
        self.sys.exit(0)
                       
    def decidingtosave(self):
        if self.saving == False or self.saving == None:
            self.saving = True
            print('self.saving = True')
        elif self.saving == True:
            self.saving = False
            print('self.saving = False')
                
    def laser_power_correction(self,n,pourcentdepart):
        
        self.percent= pourcentdepart
        self.pourcentmax = 99
        
        self.percent= self.percent/ (0.83**n) # .80 trop bas # .85 trop haut
        
        print 'self.percentactual', self.percent

        try:
            self.posrotationaryexpected = float((45./math.pi)*(math.asin(2.*(self.percent)/100.-1.)+math.pi/2.+23.*math.pi/45.))
        except:
            print 'going to 99%'
            self.exception = True
            
        if self.posrotationaryexpected  < float((45./math.pi)*(math.asin(2.*(self.pourcentmax)/100.-1.)+math.pi/2.+23.*math.pi/45.)):
            self.power_motor.goTo_abs(1,self.posrotationaryexpected) #for step of 50 microns
            #time.sleep(1)
                
        elif self.exception:
            deg=float((45./math.pi)*(math.asin(2.*99./100.-1.)+math.pi/2.+23.*math.pi/45.))
            self.power_motor.goTo_abs(1, deg)
            print 'laser maxed out at 99'
            self.exception = False
            
        
    def laserOn(self):
        self.laser.OpenLaser()
    
    def laserOff(self):
        self.laser.CloseLaser()
    
    def change_wavelength(self):
        wavelength = str(self.lineEdit_laser_output.text())
        self.laser.ChangeWavelength(wavelength)
        print 'Wavelength changed to : ', wavelength ,'nm' 
        
                
    def read_power_laser(self):
        response = self.laser.ReadPower()
        print(response, ' Watt')
        
    def read_laser_motor(self):
        response= self.laser.ReadMotor()
        print('Tune motors at ',response,' %')
        
    def move_laser_motor(self):
        motormin, motormax = self.laser.read_maxandmin_motor()
        self.read_laser_motor()
        print('motor can go from ',motormin,' to ',motormax, '. Input desired value as: nn.nn')#was 44.01 before at 803nm
        pos = raw_input()
        position = float(pos)
        try:
            self.laser.movemotor(position)
            print(position,' %')
        except:
            print('did not work')
            
#SAVING FUNCTIONS:
        
    def select_saving_directory(self):
        print('Saving button clicked!')
        options = QFileDialog.Options()
        options |= QFileDialog.DontResolveSymlinks
        options |= QFileDialog.ShowDirsOnly
        
        self.save_filename,_ = QFileDialog.getOpenFileName(self,"Choose Saving File","","HDF5 Files (*.hdf5)", options=options)

        #self.save_filename = QFileDialog.getExistingDirectory(self,"Choose Directory","", options=options)
        if self.save_filename:
            print(self.save_filename)
        print('done!')
        self.lineEdit_mouse_name.setEnabled(True)
        self.checkBox_enable_save.setEnabled(True)
        
    def update_mouse_name(self):
        self.mouse_name=self.lineEdit_mouse_name.text()
        print(self.mouse_name)
        self.checkBox_enable_save.setEnabled(True)
        
        
    #SAVING FUNCTIONS:    
        
    def set_save_folder(self):
        print('Select directory for new folder')
        options = QFileDialog.Options()
        options |= QFileDialog.DontResolveSymlinks
        options |= QFileDialog.ShowDirsOnly        
        self.save_directory = QFileDialog.getExistingDirectory(self,"Choose Directory","", options=options)
        self.lineEdit_save_name.setEnabled(True)
        self.pushButton_set_save_name.setEnabled(True)
        
    def create_new_folder(self):
        self.save_filename=str(self.lineEdit_save_name.text())+'.hdf5'
        self.lineEdit_mouse_name.setEnabled(True)
        self.checkBox_enable_save.setEnabled(True)