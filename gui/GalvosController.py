# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 10:48:31 2015

@author: flesage
"""
import pyqtgraph as pg
import datetime
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
from PyQt5.QtGui import QIntValidator
from scipy import signal
from base.Maitai import Maitai
from datetime import datetime
from scipy.io import loadmat

from base import liomio
from base.liomacq import OnDemandVoltageOutTask
from base.po2Acq import PO2Acq, PO2GatedAcq
import numpy as np
import icons_rc
from base.liomio import DataSaver
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import posixpath
from gui.ImageDisplay import po2Viewer, CurrentLineViewer
from base.Motors import ThorlabsMotor
from base.Motors import piezoClass

import ImageDisplay as imdisp
from scipy.interpolate import interp1d
import h5py

import matplotlib
matplotlib.use("Qt5Agg") #new line
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import time
import struct


import sys
sys.path.append('C:\Program Files\Alpao\SDK\Samples\Python\Lib64')
from asdk import DM

import multiprocessing
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

    changedPositionCenter = pyqtSignal(int)
    changedDurationCenter = pyqtSignal(int)
    changedPositionShort = pyqtSignal(int)
    changedDurationShort = pyqtSignal(int)
    
    changedOffsetX = pyqtSignal(int)
    changedOffsetY = pyqtSignal(int)
    changedOffsetDisplay = pyqtSignal(int)
    changedOffsetDisplayLive = pyqtSignal(int)
    
    def __init__(self):
        QWidget.__init__(self)
        basepath= os.path.join(os.path.dirname(__file__))
        uic.loadUi(os.path.join(basepath,"galvos_form_NEW.ui"), self)
        self.progressBar_powercurve.setValue(0)

        # Mechanical shutters
        self.shutter2ph_closed = True                      
        self.shutter3ph_closed = True                      
        self.pushButton_shutter_2ph.clicked.connect(self.toggle_shutter2ph)
        self.pushButton_shutter_3ph.clicked.connect(self.toggle_shutter3ph)
        self.pushButton_shutter_2ph.setStyleSheet("background-color: pale gray")
        self.pushButton_shutter_3ph.setStyleSheet("background-color: pale gray")

        self.maitaiFlag=0
        self.checkBox_PO2_channel0.clicked.connect(self.changedViewerForPO2)
        self.checkBox_PO2_channel1.clicked.connect(self.changedViewerForPO2)
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
        self.pushButton_maitai.setStyleSheet("background-color: pale gray")
        self.pushButton_maitaiShutter.setStyleSheet("background-color: pale gray")
        self.pushButton_maitaiShutter.setEnabled(False)
        self.pushButton_maitaiReadPower.clicked.connect(self.readPower_maitai)
        self.pushButton_maitaiReadPower.setStyleSheet("background-color: pale gray")
        self.pushButton_maitaiReadPower.setEnabled(False)  
        self.pushButton_maitaiReadWavelength.clicked.connect(self.readWavelength_maitai)
        self.pushButton_maitaiReadWavelength.setStyleSheet("background-color: pale gray")
        self.pushButton_maitaiReadWavelength.setEnabled(False)
        self.pushButton_maitaiSetWavelength.clicked.connect(self.setWavelength_maitai)
        self.pushButton_maitaiSetWavelength.setStyleSheet("background-color: pale gray")
        self.pushButton_maitaiSetWavelength.setEnabled(False)
        self.pushButton_maitaiCheckStatus.setStyleSheet("background-color: pale gray")
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
        self.pushButton_motor_previous_coord.clicked.connect(self.read_motor_position)
        self.pushButton_motor_home.clicked.connect(self.motors_home)
        
        
        #Stack acquisition:
        self.pushButton_start_stack.clicked.connect(self.start_stack_thread)
        self.pushButton_stop_stack.clicked.connect(self.stop_stack)   
        self.pushButton_stop.setEnabled(False)
        self.pushButton_stop_stack.setEnabled(False)
        
        #Power characterization:
        self.pushButton_powercurve_start.clicked.connect(self.start_power_curve_acquisition)
        self.pushButton_powercurve_stop.clicked.connect(self.stop_power_curve_acquisition)   
        self.pushButton_stop.setEnabled(False)
        self.pushButton_powercurve_start.setEnabled(False)        
        
        
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
        self.toggleLineFlag=0
        self.pushButton_get_line_position.clicked.connect(self.update_linescan)
        self.lineEdit_nt.textChanged.connect(self.update_linescan)
        self.lineEdit_linerate_LS.textChanged.connect(self.update_linescan)
        self.pushButton_deleteAllLines.clicked.connect(self.deleteAllLines)
        self.pushButton_start_linescan.setEnabled(True)
        self.pushButton_stop_linescan.setEnabled(False)
        self.horizontalScrollBar_line_scan_shift_y.setValue(-20)
        self.horizontalScrollBar_line_scan_shift_x.setValue(0)
        self.horizontalScrollBar_line_scan_shift_display.setValue(102)
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.update_linescan)
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.update_linescan)
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.shift_x_changed_value)
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.shift_y_changed_value)
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.shift_display_changed_value)
        self.horizontalScrollBar_shift.valueChanged.connect(self.shift_display_live_changed_value)
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.update_linescan)
        self.pushButton_start_linescan.clicked.connect(self.startlinescan)
        self.pushButton_stop_linescan.clicked.connect(self.stoplinescan)
        self.previewScanFlag = False
        self.linescan_shift_display=0
        self.diameterFlag=False
        self.toggleDiameterFlag=True
        self.pushButton_addOrthogonalLines.clicked.connect(self.toggleDiameter)
        self.diamFlag=False
        
        # PO2 acquisition
        self.po2_x_positions = None
        self.po2_y_positions = None
        self.pushButton_startPO2.clicked.connect(self.start_po2_scan)
        self.pushButton_stopPO2.clicked.connect(self.stop_po2_scan)

        self.pushButton_start_3p_po2_scan.clicked.connect(self.start_3p_po2_scan)
        self.pushButton_stop_3p_po2_scan.clicked.connect(self.stop_3p_po2_scan)

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
        self.pushButton_3PWheel_getVel.clicked.connect(self.getVel3P)
        self.pushButton_3PWheel_setVel.clicked.connect(self.setVel3P)
        
        
        self.pushButton_3Pwheel_up5.clicked.connect(self.wheel3ph_plus5)        
        self.pushButton_3Pwheel_down5.clicked.connect(self.wheel3ph_minus5)        
        self.pushButton_3Pwheel_up14.clicked.connect(self.wheel3ph_plus14)        
        self.pushButton_3Pwheel_gotomax.clicked.connect(self.wheel3ph_gotomax)        
        self.pushButton_3Pwheel_gotomin.clicked.connect(self.wheel3ph_gotomin)        

        
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
        self.checkBox_average.clicked.connect(self.toggle_averaging)
        
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
        
        self.lineEdit_linerate.setValidator(QIntValidator(0, 260))
        
        self.pushButton_fixLength.clicked.connect(self.fixLengthLines)

        self.horizontalScrollBar_LS_Radius.setMinimum(0)
        self.horizontalScrollBar_LS_Radius.setPageStep(1)
        self.horizontalScrollBar_LS_Radius.setMaximum(50.0)
        self.LS_Radius=float(self.lineEdit_LS_Radius.text())

        self.horizontalScrollBar_LS_Length.setMinimum(0)
        self.horizontalScrollBar_LS_Length.setPageStep(1)
        self.horizontalScrollBar_LS_Length.setMaximum(100.0)
        self.LS_Length=float(self.lineEdit_LS_Length.text())

        self.horizontalScrollBar_LS_Tolerance.setMinimum(0)
        self.horizontalScrollBar_LS_Tolerance.setPageStep(1)
        self.horizontalScrollBar_LS_Tolerance.setMaximum(100.0)
        self.LS_Tolerance=float(self.lineEdit_LS_Tolerance.text())/5
        
        self.pushButtonGenerateAutoScans_LS_Generate.clicked.connect(self.generateAutoLines)
        self.horizontalScrollBar_LS_Radius.valueChanged.connect(self.set_LS_Radius)
        self.horizontalScrollBar_LS_Length.valueChanged.connect(self.set_LS_Length)
        self.horizontalScrollBar_LS_Tolerance.valueChanged.connect(self.set_LS_Tolerance)
        
        self.pushButton_start_stack.setEnabled(False)
        
        self.horizontalScrollBar_PO2_xOffset.valueChanged.connect(self.set_PO2_xOffset)
        self.horizontalScrollBar_PO2_yOffset.valueChanged.connect(self.set_PO2_yOffset)
        self.horizontalScrollBar_galvo3P.valueChanged.connect(self.updateGalvo3P)
        self.pushButton_galvo3P.clicked.connect(self.toggleGalvo3P)
        self.galvo3PFlag=1
        self.horizontalScrollBar_galvo3P.setEnabled(False)
        self.init3PGalvoPos=0.0
        self.pushButton_fixPosition_galvo3P.clicked.connect(self.defineCentralPosition)
        self.centralPosition3PGalvo=0.0
        
        #move linescans
        self.pushButton_LS_up.setEnabled(False)
        self.pushButton_LS_down.setEnabled(False)
        self.pushButton_LS_left.setEnabled(False)
        self.pushButton_LS_right.setEnabled(False)
        self.pushButton_LS_up.clicked.connect(self.move_LS_up)
        self.pushButton_LS_down.clicked.connect(self.move_LS_down)
        self.pushButton_LS_left.clicked.connect(self.move_LS_left)
        self.pushButton_LS_right.clicked.connect(self.move_LS_right)
        
        #add click linescans
        self.pushButton_addLineFromPoints.clicked.connect(self.addLineFromPoints)
        self.add_line_from_points_state=True
        self.lineScanNumber=0
        self.pushButton_start_linescan.setEnabled(False)
        self.pushButtonStartMultipleLineAcq.setEnabled(False)       
        self.pushButton_startPO2.setEnabled(False)
        self.pushButton_start_3p_po2_scan.setEnabled(False)
        self.pushButton_stopPO2.setEnabled(False)
        self.pushButton_stop_3p_po2_scan.setEnabled(False)       
        self.pushButton_stop_linescan.setEnabled(False)
        self.pushButtonStopMultipleLineAcq.setEnabled(False)        
        
        #Autocorrelator:
        self.pushButton_ACMotor_InitMotor.clicked.connect(self.AC_initialize_motor)
        self.AC_motors_enabled=True
        self.pushButton_ACMotor_GoTowards.clicked.connect(self.AC_move_towards)
        self.pushButton_ACMotor_GoAway.clicked.connect(self.AC_move_away)
        self.pushButton_ACMotor_GetPosition.clicked.connect(self.AC_get_position_motor)
        self.pushButton_ACMotor_SetPeakPos.clicked.connect(self.AC_set_peak_position_motor)
        self.pushButton_ACMotor_GoPeakPos.clicked.connect(self.AC_go_to_peak_position_motor)
        self.AC_peakPosition=0.0
        
        #Display:
        self.pushButton_AutoRange.clicked.connect(self.setAutoRange)
        self.pushButton_AutoLevels.clicked.connect(self.setAutoLevels)
        self.autoRangeFlag=0
        self.autoLevelsFlag=0
        
        #PO2-LS:
        self.checkBox_activatePO2LS.clicked.connect(self.toggle_PO2_LS)
        self.simultaneousPO2LS=False
        self.horizontalScrollBar_LSPO2Power.valueChanged.connect(self.setPowerPO2_LS)
        self.horizontalScrollBar_move_center_illumination.valueChanged.connect(self.moveCenterIllumination_PO2_LS)
        self.horizontalScrollBar_change_center_illumination.valueChanged.connect(self.changeCenterIllumination_PO2_LS)
        self.horizontalScrollBar_move_short_illumination.valueChanged.connect(self.moveShortIllumination_PO2_LS)
        self.horizontalScrollBar_change_short_illumination.valueChanged.connect(self.changeShortIllumination_PO2_LS)
        self.changeCenterIllumination_PO2_LS()
        self.moveCenterIllumination_PO2_LS()
        self.changeShortIllumination_PO2_LS()
        self.moveShortIllumination_PO2_LS()
        self.eomLSFlag=0
        
        self.pushButtonUpdate3PPowerCurve.clicked.connect(self.generatePowerCurve3P)
        
    #power meter:
        self.pushButton_initPowerMeter.clicked.connect(self.initPowerMeter)
        
    #activate EOM:
        self.checkBox_activateEOM.clicked.connect(self.toggle_EOM_LS)
        self.EOM_LS_flag=0
        
    #piezo movement
        self.pushButton_piezo_on.clicked.connect(self.turnPiezoOn)
        self.pushButton_piezo_off.clicked.connect(self.turnPiezoOff)
        self.pushButton_piezo_jog.clicked.connect(self.jogPiezo)
        self.pushButton_piezo_go.clicked.connect(self.goPiezo)
        self.pushButton_piezo_stop.clicked.connect(self.stopPiezo)
        self.pushButton_piezo_oscillate.clicked.connect(self.oscillatePiezoTimer)
        self.pushButton_piezo_stop_oscillate.clicked.connect(self.stopOscillatePiezoTimer)
        self.timerPiezoFlag=0
        
    #DM:
        self.pushButton_DM_test.clicked.connect(self.test_all_actuators_DM)
        self.pushButton_DM_apply.clicked.connect(self.applyZernikeValue)
        self.pushButton_DM_reset.clicked.connect(self.resetDM)
        self.pushButton_DM_test.setEnabled(False)
        self.pushButton_DM_reset.setEnabled(False)
        self.pushButton_DM_apply.setEnabled(False)
        self.pushButton_DM_test_zernike.setEnabled(False)

        self.horizontalScrollBar_DM_tip.valueChanged.connect(self.getZernikeValue)
        self.horizontalScrollBar_DM_tilt.valueChanged.connect(self.getZernikeValue)
        self.horizontalScrollBar_DM_defocus.valueChanged.connect(self.getZernikeValue)
        self.pushButton_DM_connect.clicked.connect(self.connect_DM)
        self.pushButton_DM_test_zernike.clicked.connect(self.test_all_zernike_DM)
        
        
        self.pushButton_motorsrandomwalk.clicked.connect(self.motors_walk)
        self.pushButton_stopmotorwalk.clicked.connect(self.stop_motors_walk_thread)        
        
        
        self.tip_factor=0.0
        self.tilt_factor=0.0
        self.defocus_factor=0.0

        self.DM_connected_flag=0

        #Load Zernike polynomials:
        self.zernikePolynomials = loadmat('C:\Program Files\Alpao\SDK\Config\BAX351-Z2C.mat')
        
    def connect_DM(self):
        if not(self.DM_connected_flag):
            print("DM: Reset...")
            self.DMserialName='BAX351'
            self.dm = DM(self.DMserialName)       
            self.dm.Reset()
            print("...done!")     
            self.DM_connected_flag=1
            self.pushButton_DM_test.setEnabled(True)
            self.pushButton_DM_reset.setEnabled(True)
            self.pushButton_DM_apply.setEnabled(True)
            self.pushButton_DM_test_zernike.setEnabled(True)
        else:
            print("DM already connected!")

    def resetDM(self):
        if self.DM_connected_flag:
            print("DM: Reset...")
            self.dm.Reset()
            print("...done!")            
            self.horizontalScrollBar_DM_tip.setValue(0.0)
            self.horizontalScrollBar_DM_tilt.setValue(0.0)
            self.horizontalScrollBar_DM_defocus.setValue(0.0)
        else:
            print("DM not connected!")

            
            
    def getZernikeValue(self):
        self.tip_factor=self.horizontalScrollBar_DM_tip.value()/100.0
        self.tilt_factor=self.horizontalScrollBar_DM_tilt.value()/100.0
        self.defocus_factor=self.horizontalScrollBar_DM_defocus.value()/100.0
        print("DM: Tip "+str(self.tip_factor)+" /Tilt "+str(self.tilt_factor)+" /Defocus "+str(self.defocus_factor))
        self.lineEdit_DM_tip_value.setText(str(self.tip_factor))
        self.lineEdit_DM_tilt_value.setText(str(self.tilt_factor))
        self.lineEdit_DM_defocus_value.setText(str(self.defocus_factor))

    def applyZernikeValue(self):
        if self.DM_connected_flag:
            self.dm.Reset()
            DMcommand=self.tip_factor*self.zernikePolynomials['Z2C'][0,:]+self.tilt_factor*self.zernikePolynomials['Z2C'][1,:]+self.defocus_factor*self.zernikePolynomials['Z2C'][2,:]
            self.dm.Send(DMcommand.tolist())
            print("Applied Zernike")
        else:
            print("DM not connected!")
            
        
    def test_all_actuators_DM(self):
        if self.DM_connected_flag:
            print("Retrieve number of actuators")
            nbAct = int( self.dm.Get('NBOfActuator') )
            print( "Number of actuator for " + self.DMserialName + ": " + str(nbAct) )
    
            print("Send 0 on each actuators")
            values = [0.] * nbAct
            self.dm.Send( values )
    
            print("We will send on all actuator 10% for 1/2 second")
            for i in range( nbAct ):
                values[i] = 0.50
                # Send values vector
                self.dm.Send( values )
                print("Send 0.10 on actuator " + str(i))
                time.sleep(0.5) # Wait for 0.5 second
                values[i] = 0
    
                print("Reset")
                self.dm.Reset()
    
            print("Exit")
        else:
            print("DM not connected!")
            
    def test_all_zernike_DM(self):
        if self.DM_connected_flag:
            print("Retrieve number of actuators")
            nbAct = int( self.dm.Get('NBOfActuator') )
            print( "Number of actuator for " + self.DMserialName + ": " + str(nbAct) )
    
            print("Send 0 on each actuators")
            values = [0.] * nbAct
            self.dm.Send( values )
    
            print("We will 15 first modes Zernike")
            for i in range( 15 ):
                DMcommand=0.5*self.zernikePolynomials['Z2C'][i,:]
                # Send values vector
                self.dm.Send( DMcommand)
                print("Send 0.10 on mode " + str(i))
                time.sleep(1.0) # Wait for 0.5 second
                    
                print("Reset")
                self.dm.Reset()
    
            print("Exit")
        else:
            print("DM not connected!")

        
    def oscillatePiezoTimer(self):
        self.timerPiezo = pg.QtCore.QTimer()
        time=int(self.lineEdit_piezo_time_per_iter.text())
        speed=self.spinBox_piezo.value()
        self.piezo.setSpeed(speed)
        self.timerPiezo.timeout.connect(self.piezo.piezoOscillateUpdate)
        self.timerPiezoFlag=1
        self.timerPiezo.start(time*1000)
        
        
    def stopOscillatePiezoTimer(self):
        if self.timerPiezoFlag:
            self.timerPiezo.stop()
            self.stopPiezo()
        
    def oscillatePiezo(self):
        print('piezo oscillate')
        speed=self.spinBox_piezo.value()
        time=int(self.lineEdit_piezo_time_per_iter.text())
        num=int(self.lineEdit_piezo_number_iter.text())
        self.piezo.piezoOscillate(num,time,speed)
        
    def turnPiezoOn(self):
        self.piezo=piezoClass('COM7',19200)
        print('piezo turned on')

    def turnPiezoOff(self):
        self.piezo.closeMotor()
        print('piezo turned off')
        
    def jogPiezo(self):
        speed=self.spinBox_piezo.value()
        self.piezo.startJog(speed)      
        print('jogging started with speed: '+str(speed))
        
    def goPiezo(self):
        distance=float(self.lineEdit_piezo_distance.text())
        self.piezo.goToPosition(distance)     
        print('motor started with distance: '+str(distance))

    def stopPiezo(self):
        self.piezo.stopMotor()
        print('piezo stopped')

    def stopOscillation(self):
        self.piezo.setStopFlag(0)
        
        
    def generatePowerCurve2P(self):
        perc=np.arange(0,100,5)
        val=[1.3,8,23,45,75,113,155,204,257,312,370,426,484,580,580,580,580,580,580,580]
        f2 = interp1d(perc, val, kind='quadratic',fill_value='extrapolate')
        perc2=np.arange(0,100,1);
        self.powerValues2P=f2(perc2)
        
    def generatePowerCurve3P(self):
        perc=np.array([0,15,23,31,38,46,54,62,69,77,85,92,100])
        val=np.array([0.6,7,16.3,29.3,44,61,79,97,114,129,141,150,155])
        minVal=float(self.lineEdit_3PWheelMin.text())
        maxVal=float(self.lineEdit_3PWheelMax.text())
        val=(val-min(val))/max(val)*(maxVal)
        val=val+minVal
        
        f2 = interp1d(perc, val, kind='quadratic',fill_value='extrapolate')
        perc2=np.arange(0,100,1);
        self.powerValues3P=f2(perc2)
        
        
    def initPowerMeter(self):        
        
        import serial
        
        import visa
        from ThorlabsPM100 import ThorlabsPM100
        rm = visa.ResourceManager()
        print(rm.list_resources()) #2


        inst = rm.open_resource('USB0::4883::32888::P0021056::0::INSTR', write_termination='\r', read_termination='\r')# ,term_chars='\n', timeout=1)
        
        inst.query('*IDN?')
        self.power_meter = ThorlabsPM100(inst=inst)

    def setInitialPower(self):
        power2ph=self.horizontalScrollBar_power2ph.value()
        self.setPower2ph(power2ph)

    #        
    def setPowerPO2_LS(self,value):
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        self.label_powerPO2LS.setText(str(int(value))+' %')

    def updateParameters_EOM_LS(self):
        self.eomLSFlag=self.checkBox_activateEOM.isChecked()
        power2ph=self.horizontalScrollBar_power2ph.value()
        powerPO2=0.0
        powerLS = 2.0*power2ph/100.0

        onTimePositionShort = 0.0
        on_time = 0.0
        lineRate=float(self.lineEdit_linerate_LS.text())
        nr=float(self.lineEdit_nr.text())
        n_extra=float(self.lineEdit_extrapoints_LS.text())
        daq_freq=lineRate*(nr+n_extra)
        lineModulation=self.checkBox_modulateAllLinesEOM.isChecked()
        if lineModulation:
            onTimeRatioCenter = 0.03
            onTimePositionCenter = 95        
        else:
            onTimeRatioCenter = 0.3
            onTimePositionCenter = 95      
        self.galvos.setEOMParameters(self.eomLSFlag,config.eom_ao,daq_freq,powerPO2,powerLS,on_time,onTimePositionCenter,onTimeRatioCenter,onTimePositionShort,lineModulation)
        return powerLS, powerPO2, onTimeRatioCenter, onTimePositionCenter, onTimePositionShort, on_time
        
    def toggle_EOM_LS(self):
        print('Toggling PO2-LS...')
        self.eomLSFlag=self.checkBox_activateEOM.isChecked()
        if self.eomLSFlag:
            print('...Activating PO2-LS')
            self.updateParameters_EOM_LS()
        else:
            print('...Deactivating PO2-LS')
            self.galvos.setEOMFlag(self.eomLSFlag)   
    
    def toggle_PO2_LS(self):
        print('Toggling PO2-LS...')
        self.simultaneousPO2LS=self.checkBox_activatePO2LS.isChecked()
        if self.simultaneousPO2LS:
            print('...Activating PO2-LS')
            self.updateParametersSimultaneous_PO2LS()
        else:
            print('...Deactivating PO2-LS')
            self.galvos.setEOMFlag(self.simultaneousPO2LS)
    
    def changeCenterIllumination_PO2_LS(self):
        ratio=self.getCentralIlluminationRatio_PO2LS()
        self.lineEdit_dur_center_illumination.setText(str(ratio))
    
    def moveCenterIllumination_PO2_LS(self):
        pos=self.getCentralIlluminationPosition_PO2LS()
        self.lineEdit_pos_center_illumination.setText(str(pos))        
        
    def moveShortIllumination_PO2_LS(self):
        pos=self.getShortIlluminationPosition_PO2LS()
        self.lineEdit_pos_short_illumination.setText(str(pos))    
        
    def changeShortIllumination_PO2_LS(self):
        on_time = self.horizontalScrollBar_change_short_illumination.value()
        self.lineEdit_dur_short_illumination.setText(str(on_time))    
    
    def getCentralIlluminationRatio_PO2LS(self):
        onTimeRatioCenter=self.horizontalScrollBar_change_center_illumination.value()
        onTimeRatioCenter=onTimeRatioCenter/100.0
        return onTimeRatioCenter
    
    def getCentralIlluminationPosition_PO2LS(self):
        onTimePositionCenter=self.horizontalScrollBar_move_center_illumination.value()
        return onTimePositionCenter

    def getShortIlluminationPosition_PO2LS(self):
        onTimePositionShort=self.horizontalScrollBar_move_short_illumination.value()
        return onTimePositionShort
    
    def updateParametersSimultaneous_PO2LS(self):
        self.simultaneousPO2LS=self.checkBox_activatePO2LS.isChecked()
        power2ph=self.horizontalScrollBar_power2ph.value()
        powerPO2=self.horizontalScrollBar_LSPO2Power.value()
        powerLS = 2.0*power2ph/100.0
        powerPO2 = 2.0*powerPO2/100.0
        onTimeRatioCenter = self.getCentralIlluminationRatio_PO2LS()
        onTimePositionCenter = self.getCentralIlluminationPosition_PO2LS()
        onTimePositionShort = self.getShortIlluminationPosition_PO2LS()
        on_time = self.horizontalScrollBar_change_short_illumination.value()
        self.galvos.setEOMParameters(self.simultaneousPO2LS,config.eom_ao,0.6e6,powerPO2,powerLS,on_time,onTimePositionCenter,onTimeRatioCenter,onTimePositionShort,1)
        return powerLS, powerPO2, onTimeRatioCenter, onTimePositionCenter, onTimePositionShort, on_time
          
    def setAutoRange(self):
        if (self.autoRangeFlag):
            self.pushButton_AutoRange.setText('Auto Range: OFF')
            self.pushButton_AutoRange.setStyleSheet("background-color: gray")
            self.autoRangeFlag=0
        else:
            self.pushButton_AutoRange.setText('Auto Range: ON')
            self.pushButton_AutoRange.setStyleSheet("background-color: yellow")
            self.autoRangeFlag=1  
        self.viewer.setAutoRange()
        self.viewer2.setAutoRange()
        
    def setAutoLevels(self):
        if (self.autoRangeFlag):
            self.pushButton_AutoLevels.setText('Auto Levels: OFF')
            self.pushButton_AutoLevels.setStyleSheet("background-color: gray")
            self.autoRangeFlag=0
        else:
            self.pushButton_AutoLevels.setText('Auto Levels: ON')
            self.pushButton_AutoLevels.setStyleSheet("background-color: yellow")
            self.autoRangeFlag=1 
        self.viewer.setAutoLevels()
        self.viewer2.setAutoLevels() 
           
    def initialize_triangular_display(self):
        self.viewer.getScanningType(self.comboBox_scantype.currentText())
        self.viewer2.getScanningType(self.comboBox_scantype.currentText())
        self.viewer.move_offset_display_live(self.horizontalScrollBar_shift.value())
        self.viewer2.move_offset_display_live(self.horizontalScrollBar_shift.value())
    #Autocorrelator functions:
    
    def AC_go_to_peak_position_motor(self):
        self.AC_peakPosition=float(self.lineEdit_ACMotor_PeakPos.text())
        print('*** Autocorrelator: go to peak...')
        currentPosition=self.AC_get_position_motor()
        self.AC_motor.setpos(self.AC_peakPosition)
        currentPositionAfter=self.AC_get_position_motor()
        print('*** done!')          
        print('*** Autocorrelator: moved of '+str(currentPositionAfter-currentPosition)) 
        
    def AC_set_peak_position_motor(self):
        self.AC_peakPosition=self.AC_get_position_motor()
        self.lineEdit_ACMotor_PeakPos.setText(str(float(int(self.AC_peakPosition*100))/100.0))   

    def AC_initialize_motor(self):
        if self.AC_motors_enabled:
            print('*** Autocorrelator: turn motor ON...')
            SN=27000620
            HWTYPE=27
            self.AC_motor=ThorlabsMotor(SN,HWTYPE)
            ACmotorStatus=True
        else:
            print('*** Autocorrelator: turn motor OFF...')
            ACmotorStatus=False
            
        self.pushButton_ACMotor_GoAway.setEnabled(ACmotorStatus)
        self.pushButton_ACMotor_GoTowards.setEnabled(ACmotorStatus)
        self.pushButton_ACMotor_GetPosition.setEnabled(ACmotorStatus)
        self.pushButton_ACMotor_GoPeakPos.setEnabled(ACmotorStatus)
        self.pushButton_ACMotor_SetPeakPos.setEnabled(ACmotorStatus)
        self.lineEdit_ACMotor_StepSize.setEnabled(ACmotorStatus)
        self.AC_set_peak_position_motor()
        self.AC_motors_enabled=not(ACmotorStatus)  
        print('*** done!')          
        
    def AC_clean_motor(self):
        print('*** Autocorrelator: clearing motors...')
        try:
            self.AC_motor.clean_up_APT()
        except AttributeError:
            print('*** Autocorrelator: already cleared!')
        else:
            del(self.AC_motor)
        print('*** done!')       
        
    def AC_get_position_motor(self):
        currentPosition=self.AC_motor.getpos()
        self.lineEdit_ACMotor_CurrentPos.setText(str(float(int(currentPosition*100))/100.0))   
        return currentPosition
        
    def AC_move_towards(self):
        print('*** Autocorrelator: moving towards detector...')
        currentPosition=self.AC_get_position_motor()
        step=float(self.lineEdit_ACMotor_StepSize.text())
        self.AC_motor.setposRel(step)
        currentPositionAfter=self.AC_get_position_motor()
        print('*** done!')          
        print('*** Autocorrelator: moved of'+str(currentPositionAfter-currentPosition))     
        
    def AC_move_away(self):
        print('*** Autocorrelator: moving away detector...')
        currentPosition=self.AC_get_position_motor()
        step=float(self.lineEdit_ACMotor_StepSize.text())
        self.AC_motor.setposRel(-step)
        currentPositionAfter=self.AC_get_position_motor()
        print('*** done!')          
        print('*** Autocorrelator: moved of'+str(currentPositionAfter-currentPosition))     

    def AC_go_home(self):
        print('*** Autocorrelator: going home...')
        currentPosition=self.AC_motor.getpos()
        self.lineEdit_ACMotor_CurrentPos.setText(str(currentPosition))
        self.AC_motor.goHome()
        currentPositionAfter=self.AC_motor.getpos()
        self.lineEdit_ACMotor_CurrentPos.setText(str(currentPositionAfter))   
        print('*** done!')                 
        
    #Creates lines from points:
    def addLineFromPoints(self):
        print('adding first point')
        self.toggle_add_line_from_points_state()
        self.updateNumberOfLines()
        if self.numberOfLines>0:
            self.enableMoveButtons(True)

    def toggle_add_line_from_points_state(self):
        if (self.add_line_from_points_state):
            self.pushButton_addLineFromPoints.setText('Stop adding')
            self.add_line_from_points_state=False
            self.viewer.updateGenerateLineFromPointFlag(True)
        else:
            self.pushButton_addLineFromPoints.setText('Add points')
            self.add_line_from_points_state=True
            self.viewer.updateGenerateLineFromPointFlag(False)


    #autolineScans:
    def enableMoveButtons(self,val):
        self.pushButton_LS_up.setEnabled(val)
        self.pushButton_LS_down.setEnabled(val)
        self.pushButton_LS_left.setEnabled(val)
        self.pushButton_LS_right.setEnabled(val)  
        
    def move_LS_up(self):
        step=float(self.lineEdit_LS_shift.text())
        print(step)
        self.viewer.shiftLines(0,-step)
        print('moving line up')
        
    def move_LS_down(self):
        step=float(self.lineEdit_LS_shift.text())
        self.viewer.shiftLines(0,step)
        print('moving line down')
        print(step)

    def move_LS_left(self):
        step=float(self.lineEdit_LS_shift.text())
        self.viewer.shiftLines(-step,0)
        print('moving line left')
        print(step)
      
    def move_LS_right(self):
        step=float(self.lineEdit_LS_shift.text())
        self.viewer.shiftLines(step,0)
        print('moving line right')
        print(step)

        
    def generateAutoLines(self):
        print(self.LS_Tolerance)

        self.viewer.generateAutoLines(diam=self.LS_Radius, 
                                      length=self.LS_Length, 
                                      tolerance=self.LS_Tolerance)
        self.viewer.displayLines()
        
        
    def set_LS_Radius(self):
        self.LS_Radius=float(self.horizontalScrollBar_LS_Radius.value())
        self.lineEdit_LS_Radius.setText(str(self.LS_Radius))

    def set_LS_Length(self):
        self.LS_Length=float(self.horizontalScrollBar_LS_Length.value())
        self.lineEdit_LS_Length.setText(str(self.LS_Length))
        
    def set_LS_Tolerance(self):
        
        def valueHandler(value):   
            return float(value)/500

        self.LS_Tolerance=valueHandler(self.horizontalScrollBar_LS_Tolerance.value())
        self.lineEdit_LS_Tolerance.setText(str(self.LS_Tolerance*5))
        print(self.LS_Tolerance)

    #autocorrelator:
    def setAiAPD(self,ai_APD):
        self.ai_APD=ai_APD
        daq_freq=10e5
        self.ai_APD.config(10,daq_freq,True)
        
        
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
        self.pushButton_3Pwheel_down5.setEnabled(status)
        self.pushButton_3Pwheel_up5.setEnabled(status)
        self.pushButton_3Pwheel_gotomin.setEnabled(status)
        self.pushButton_3Pwheel_gotomax.setEnabled(status)
        self.pushButton_3Pwheel_up14.setEnabled(status)

        
    def setVel3P(self):
        self.ThreePWheelVelocity=float(self.lineEdit_3PWheel_vel.text())
        self.thorlabs.setVelocityParameters(0.0,10.0,self.ThreePWheelVelocity)
        param=self.thorlabs.getVelocityParameterLimits()
        print(param)

    def getVel3P(self):
        param=self.thorlabs.getVelocityParameters()
        print(param)
        
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
    def defineViewerForPO2(self):
        flagChannel0=self.checkBox_PO2_channel0.isChecked()
        flagChannel1=self.checkBox_PO2_channel1.isChecked()
        
        if ((flagChannel0==1) & (flagChannel1==0)):
            self.viewerPO2=self.viewer
            self.viewerTxt='viewer'
        elif ((flagChannel0==0) & (flagChannel1==1)):
            self.viewerPO2=self.viewer2
            self.viewerTxt='viewer'
        else:
            print('no channel for PO2 rectangle displayed')
            self.viewerPO2=self.viewer
            self.viewerTxt='viewer'
            
    def changedViewerForPO2(self):
        self.clearGrid_PO2()
        self.defineViewerForPO2()
        self.clearGrid_PO2()
    
    
    def updateViewerConsumer(self,flag):
        self.defineViewerForPO2()
        self.ai_task.updateConsumerFlag(self.viewerTxt,flag)
    
    def po2_add_point(self):
        self.toggle_po2_add_point_state()
        
    def toggle_po2_add_point_state(self):
        if (self.po2_add_point_state):
            self.pushButton_po2AddPoint.setText('Stop adding points')
            self.po2_add_point_state=False
            self.viewerPO2.updateGeneratePointFlag(True)
        else:
            self.pushButton_po2AddPoint.setText('Add Points')
            self.po2_add_point_state=True
            self.viewerPO2.updateGeneratePointFlag(False)
    
    def defineROI_PO2(self):
        self.viewerPO2.resetRect()
        nx=float(self.lineEdit_nx.text())
        ny=float(self.lineEdit_ny.text())
        self.viewerPO2.createRectangle(nx,ny)
        self.viewerPO2.displayRectangles()
    
    def set_PO2_xOffset(self):
        self.PO2_xOffset=self.horizontalScrollBar_PO2_xOffset.value()
        self.lineEdit_PO2_xOffset.setText(str(self.PO2_xOffset))
    
    def set_PO2_yOffset(self):
        self.PO2_yOffset=self.horizontalScrollBar_PO2_yOffset.value()
        self.lineEdit_PO2_yOffset.setText(str(self.PO2_yOffset))    

    
    def generateGrid_PO2(self):
        [x_o,y_o,width,height]=self.viewerPO2.getMouseSelectedRectPosition()
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
                    self.viewerPO2.createPoint(i,j)
            self.viewerPO2.displayPoints()
            self.viewerPO2.resetRect()
            
    def convertPosToVolts(self):
        self.nx=float(self.lineEdit_nx.text())
        self.ny=float(self.lineEdit_ny.text())
        self.n_extra=float(self.lineEdit_extrapoints.text())
        self.width=float(self.lineEdit_width.text())
        self.height=float(self.lineEdit_height.text())
         
        [self.x_pos,self.y_pos]=self.viewerPO2.getPositionPoints()
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
        self.set_PO2_xOffset()
        self.set_PO2_yOffset()
        self.po2_x_positions = self.PO2_xOffset+x_pos_grid
        self.po2_y_positions = self.PO2_yOffset+y_pos_grid
        
    def toggleGalvo3P(self):
        if self.galvo3PFlag:
            self.activateGalvo3P()
            self.pushButton_galvo3P.setText('Disable galvo')
            self.horizontalScrollBar_galvo3P.setEnabled(True)
            self.pushButton_fixPosition_galvo3P.setEnabled(True)
            self.lineEdit_fixPosition_galvo3P.setEnabled(True)
            self.galvo3PFlag=0
        else:
            self.disableGalvo3P()
            self.pushButton_galvo3P.setText('Activate galvo')
            self.horizontalScrollBar_galvo3P.setEnabled(False)
            self.pushButton_fixPosition_galvo3P.setEnabled(False)
            self.lineEdit_fixPosition_galvo3P.setEnabled(False)
            self.galvo3PFlag=1           
        
    def activateGalvo3P(self):
        print('Activate Galvo 3P... ')
        try:
            self.galvo_gate_task.close()
        except AttributeError:
            print('*** 3P Galvo gate: already cleared!')
        else:
            del(self.galvo_gate_task)
        
        self.horizontalScrollBar_galvo3P.setEnabled(False)
        self.galvo_gate_task = PO2GatedAcq(config.gated_device, config.gated_ao)
        self.galvo_gate_task.configOnDemand()   
        self.galvo_gate_task.moveOnDemand(self.init3PGalvoPos)
        self.horizontalScrollBar_galvo3P.setValue(self.init3PGalvoPos)
        self.lineEdit_galvo3P.setText(str(self.init3PGalvoPos))

        print(' ...done!')
        
    def setGalvoToPos3P(self,pos):
        print('Activate Galvo 3P... ')
        #try:
        #    self.galvo_gate_task.close()
        #except AttributeError:
        #    print('*** 3P Galvo gate: already cleared!')
        #else:
            #del(self.galvo_gate_task)
        self.galvo_gate_task = PO2GatedAcq(config.gated_device, config.gated_ao)
        self.galvo_gate_task.configOnDemand()   
        self.galvo_gate_task.moveOnDemand(pos)
        print('Disable Galvo 3P... ')
        try:
            self.galvo_gate_task.close()
        except AttributeError:
            print('*** 3P Galvo gate: already cleared!')
        #else:
            #del(self.galvo_gate_task)  
        print(' ...done!')            
            
        
    def disableGalvo3P(self):
        self.galvo_gate_task.moveOnDemand(self.init3PGalvoPos)

        print('Disable Galvo 3P... ')
        try:
            self.galvo_gate_task.close()
        except AttributeError:
            print('*** 3P Galvo gate: already cleared!')
        else:
            del(self.galvo_gate_task)  
        print(' ...done!')      

    def updateGalvo3P(self):
        position3P=float(self.horizontalScrollBar_galvo3P.value())
        print('3P Galvo moving to... '+str(position3P/1000))
        self.lineEdit_galvo3P.setText(str(position3P/1000))
        self.galvo_gate_task.moveOnDemand(position3P)
        print(' ...done!')
        
    def defineCentralPosition(self):
        self.centralPosition3PGalvo=float(self.horizontalScrollBar_galvo3P.value())/1000
        self.lineEdit_fixPosition_galvo3P.setText(str(self.centralPosition3PGalvo))

    def modifyGrid_PO2(self):
        self.viewerPO2.removeAllPoints()
        self.numberOfXPoints=self.horizontalScrollBar_xPoints.value()
        self.numberOfYPoints=self.horizontalScrollBar_yPoints.value()
        self.lineEdit_numXpoints.setText(str(self.numberOfXPoints))
        self.lineEdit_numYpoints.setText(str(self.numberOfYPoints))
        self.generateGrid_PO2()
        
    def clearGrid_PO2(self):
        self.viewerPO2.removeAllPoints()
        self.viewerPO2.resetRect()
        
    def defineChannelToPlot(self):
        flagChannel0=self.checkBox_PO2_disp_channel0.isChecked()
        flagChannel1=self.checkBox_PO2_disp_channel1.isChecked()
        
        if ((flagChannel0==1) & (flagChannel1==0)):
            self.channelToPlotPO2=0
        elif ((flagChannel0==0) & (flagChannel1==1)):
            self.channelToPlotPO2=1
        else:
            print('no channel for PO2 display')
            self.channelToPlotPO2=1
        
        
    def start_po2_scan(self):
        self.checkBox_activateEOM.setChecked(0)
        self.eomLSFlag=0
        self.defineChannelToPlot()
        flagPreviousTraces=self.checkBox_PO2_showPreviousTraces.isChecked()
        self.convertPosToVolts()
        self.showPO2Viewer()
        self.ai_task.updateConsumerFlag(self.viewerTxt,False)        
        
        power2ph=self.horizontalScrollBar_power2ph.value()
        
        self.setPower2ph(0)
        
        print('Starting po2 acquisition:')
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
        self.freqPO2=5e5
        n_averages = int(self.lineEdit_n_po2_averages.text())
        self.po2viewer.getAcquisitionParameters(n_averages,gate_on,gate_off,self.freqPO2)
        self.po2viewer.showPreviousTraces(flagPreviousTraces)
        
        desiredNumTracesToPlot=int(self.lineEdit_n_po2_previousTraces.text())
        numberOfPlots=self.po2_x_positions.shape[0]
        
        if (numberOfPlots<desiredNumTracesToPlot):
            tracesToPlot=desiredNumTracesToPlot
        else:
            tracesToPlot=numberOfPlots
                
        self.po2viewer.initPlot(tracesToPlot)

        self.eom_task = PO2Acq(config.eom_device, config.eom_ao, gate_on, gate_off, voltage_on, n_averages,self.freqPO2)
        self.eom_task.setSynchronizedAITask(self.ai_task)
        self.eom_task.config()
        
        # Acquire list of points to move to
        self.galvos.configOnDemand()
        self.ai_task.setDecoder(None)
        self.ai_task.setDataConsumer(self.po2viewer,True,self.channelToPlotPO2,'po2plot',True)
        #
        self.get_current_z_depth()
        self.toggle_shutter2ph()
        time.sleep(1)
        # Loop over points by using the aoDoneSignal, start the first point manually
        self.i_po2_point = -1
        self.eom_task.po2_task.signal_helper.aoDoneSignal.connect(self.nextPO2Point)        
        
        #saving parameters:
        nx_preview=int(self.lineEdit_nx.text())
        ny_preview=int(self.lineEdit_ny.text())
        width=float(self.lineEdit_width.text())
        height=float(self.lineEdit_height.text())
        self.gate_on_2P_PO2=gate_on
        self.gate_off_2P_PO2=gate_on
        self.freq_2P_PO2=self.freqPO2
        self.voltage_on_2P_PO2=voltage_on
        self.n_averages_2P_PO2=n_averages
        if self.checkBox_enable_save.isChecked():
            #if(self.i_po2_point>0):
            #   print('stop saving...')
            #  self.data_saver.stopSaving()
            # self.ai_task.removeDataConsumer(self.data_saver)
                
            print('in save...')
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'po2plot'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            self.po2PlotNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_po2plot_acq.setText(str(self.po2PlotNumber))
            self.scanNumber = self.scanType+'_'+str(self.po2PlotNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            self.data_saver.setDatasetName(self.pathName) 
            #self.data_saver.addAttribute('meas num:',self.i_po2_point+1)  
            self.data_saver.addAttribute('total meas num:',self.po2_x_positions.shape[0])  
            self.data_saver.addAttribute('x_pos',self.po2_x_positions)            
            self.data_saver.addAttribute('y_pos',self.po2_y_positions)              
            self.data_saver.addAttribute('depth',self.currentZPos)
            self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
            self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
            self.data_saver.addAttribute('last live scan:',self.liveScanNumber)
            self.data_saver.addAttribute('last stack acq:',self.stackNumber)
            self.data_saver.addAttribute('gate on [us]:',self.gate_on_2P_PO2)
            self.data_saver.addAttribute('gate off [us]:',self.gate_off_2P_PO2)
            self.data_saver.addAttribute('amplitude voltage ON:',self.voltage_on_2P_PO2)
            self.data_saver.addAttribute('freq acq. PO2 [Hz]:',self.freq_2P_PO2)
            self.data_saver.addAttribute('n averages:',self.n_averages_2P_PO2)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)                 
            self.data_saver.addAttribute('nx',nx_preview)
            self.data_saver.addAttribute('ny',ny_preview)  
            self.data_saver.addAttribute('x_offset',self.PO2_xOffset)       
            self.data_saver.addAttribute('y_offset',self.PO2_yOffset)       
            self.comment=(self.lineEdit_comment.text())
            self.data_saver.addAttribute('comment',self.comment)                
            self.data_saver.setBlockSize(512)          
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            self.data_saver.startSaving()        
        self.viewerPO2.initPO2Calculation(self.checkBox_PO2_fitData.isChecked())
        
        if (self.checkBox_PO2_fitData.isChecked()):
            lowBoundary=10
            upBoundary=80
        else:
            lowBoundary=17
            upBoundary=150    
        self.label_PO2_lowBoundary.setText(str(lowBoundary))
        self.label_PO2_upperBoundary.setText(str(upBoundary))
        
        temperature=float(self.lineEdit_n_po2_temperature.text())
        self.po2viewer.initCalculation(self.freqPO2,self.checkBox_PO2_fitData.isChecked(),temperature)
        self.po2viewer.shiftDisplay(0,0)
        self.nextPO2Point()

    def start_3p_po2_scan(self):
        self.checkBox_activateEOM.setChecked(0)
        self.eomLSFlag=0
        self.defineChannelToPlot()
        flagPreviousTraces=self.checkBox_PO2_showPreviousTraces.isChecked()

        voltage_on = float(self.lineEdit_po2_galvo_amplitude.text())
        offset = float(self.lineEdit_po2_galvo_offset.text())
        print('Starting 3P po2 acquisition:')

        pos=-3
        print(pos)
        self.setGalvoToPos3P(pos)
        self.convertPosToVolts()        
        self.showPO2Viewer()
        self.ai_task.updateConsumerFlag(self.viewerTxt,False)        
        power3ph=self.horizontalScrollBar_power3ph.value()
        # Here we want to block until the power is reached since we need all curves to be taken with the same 
        # power
        #self.setPower3ph_noThread(power3ph)

        # Steps
        galvo_freq = float(self.lineEdit_po2_galvo_freq.text())
        n_averages = int(self.lineEdit_n_po2_averages.text())

        self.freqPO2=5e5
        self.po2viewer.getAcquisitionParameters3P(n_averages,galvo_freq,self.freqPO2)
        desiredNumTracesToPlot=int(self.lineEdit_n_po2_previousTraces.text())
        numberOfPlots=self.po2_x_positions.shape[0]
        
        if (numberOfPlots<desiredNumTracesToPlot):
            tracesToPlot=desiredNumTracesToPlot
        else:
            tracesToPlot=numberOfPlots
                
        self.po2viewer.initPlot(tracesToPlot)
        self.galvo_gate_task = PO2GatedAcq(config.gated_device, config.gated_ao)
        self.galvo_gate_task.setSynchronizedAITask(self.ai_task)
        self.galvo_gate_task.config(galvo_freq, voltage_on, offset, n_averages,self.freqPO2)
        
        # Acquire list of points to move to
        self.galvos.configOnDemand()
        self.ai_task.setDecoder(None)
        self.ai_task.setDataConsumer(self.po2viewer,True,1,'po2plot',True)
        self.get_current_z_depth()
        self.toggle_shutter3ph()

        nx_preview=int(self.lineEdit_nx.text())
        ny_preview=int(self.lineEdit_ny.text())
        width=float(self.lineEdit_width.text())
        height=float(self.lineEdit_height.text())


        # Loop over points by using the aoDoneSignal, start the first point manually
        self.i_po2_point = -1
        self.galvo_gate_task.po2_task.signal_helper.aoDoneSignal.connect(self.next3PPO2Point)
        
        if self.checkBox_enable_save.isChecked():
            #if(self.i_po2_point>0):
            #   print('stop saving...')
            #  self.data_saver.stopSaving()
            # self.ai_task.removeDataConsumer(self.data_saver)
                
            print('in save...')
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = '3P-po2plot'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            self.po2PlotNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_po2plot_acq.setText(str(self.po2PlotNumber))
            self.scanNumber = self.scanType+'_'+str(self.po2PlotNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            self.data_saver.setDatasetName(self.pathName) 
            #self.data_saver.addAttribute('meas num:',self.i_po2_point+1)  
            self.data_saver.addAttribute('total meas num:',self.po2_x_positions.shape[0])  
            self.data_saver.addAttribute('x_pos',self.po2_x_positions)            
            self.data_saver.addAttribute('y_pos',self.po2_y_positions)              
            self.data_saver.addAttribute('depth',self.currentZPos)
            self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
            self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
            self.data_saver.addAttribute('last live scan:',self.liveScanNumber)
            self.data_saver.addAttribute('last stack acq:',self.stackNumber)
            self.data_saver.addAttribute('galvo. gate offset:',offset)
            self.data_saver.addAttribute('amplitude voltage GALVO:',voltage_on)
            self.data_saver.addAttribute('freq acq. PO2 [Hz]:',self.freqPO2)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)                 
            self.data_saver.addAttribute('nx',nx_preview)
            self.data_saver.addAttribute('ny',ny_preview)  
            self.data_saver.addAttribute('galvo_freq',galvo_freq)       
            self.data_saver.addAttribute('n_averages',n_averages)
            self.comment=(self.lineEdit_comment.text())
            self.data_saver.addAttribute('comment',self.comment)
            val=self.horizontalScrollBar_power3ph.value()
            self.data_saver.addAttribute('3Pperc',val)
            
 
                
            self.data_saver.setBlockSize(512)          
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            self.data_saver.startSaving()        
        self.viewerPO2.initPO2Calculation(self.checkBox_PO2_fitData.isChecked())
        
        if (self.checkBox_PO2_fitData.isChecked()):
            lowBoundary=10
            upBoundary=80
        else:
            lowBoundary=17
            upBoundary=150    
        self.label_PO2_lowBoundary.setText(str(lowBoundary))
        self.label_PO2_upperBoundary.setText(str(upBoundary))
        
        temperature=float(self.lineEdit_n_po2_temperature.text())
        self.po2viewer.initCalculation(self.freqPO2,self.checkBox_PO2_fitData.isChecked(),temperature)
        val=int(self.lineEdit_po2_3P_display.text())
        self.po2viewer.shiftDisplay(1,val)
        
        self.next3PPO2Point()
  
    @pyqtSlot()       
    def nextPO2Point(self):
        time.sleep(0.5)
        #self.shutter2ph.off() 
        #self.shutter2ph_closed = True
        #time.sleep(1)
        #self.pushButton_shutter_2ph.setStyleSheet("background-color: pale gray");
        #self.pushButton_shutter_2ph.setText('Shutter 2Ph: Closed')        
        print('in next point!')
        self.i_po2_point = self.i_po2_point+1
        print(self.i_po2_point)
        if (self.i_po2_point>1):
            estPO2=self.po2viewer.getEstimationPO2()
            self.viewerPO2.characPoint(self.i_po2_point-2,estPO2)
            self.viewerPO2.highlightPoint(self.i_po2_point)
        else:
            self.viewerPO2.highlightPointAll(self.i_po2_point)
        if(self.i_po2_point < self.po2_x_positions.shape[0]):
            #    Start acquisition task averaging doing a finite ao task reapeated n average times
            #    Show in a plot the Decay curve
            #   Should remove these prints once we know it works. No need to update viewer anymore
            print('PO2 measurement: ' + str(self.i_po2_point+1) + ' out of ' + str(self.po2_x_positions.shape[0]))
            print(str(self.po2_x_positions[self.i_po2_point])+';'+str(self.po2_y_positions[self.i_po2_point]))
            #
#             if self.checkBox_enable_save.isChecked():
#                 if(self.i_po2_point>0):
#                     print('stop saving...')
#                     self.data_saver.stopSaving()
#                     self.ai_task.removeDataConsumer(self.data_saver)
#                 
#                 print('in save...')
#                 self.data_saver=DataSaver(self.save_filename)
#                 self.mouseName=self.lineEdit_mouse_name.text()
#                 self.scanType = 'po2plot'      
#                 self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
#                 self.po2PlotNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
#                 self.lineEdit_po2plot_acq.setText(str(self.po2PlotNumber))
#                 self.scanNumber = self.scanType+'_'+str(self.po2PlotNumber) 
#                 self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
#                 self.data_saver.setDatasetName(self.pathName) 
#                 self.data_saver.addAttribute('meas num:',self.i_po2_point+1)  
#                 self.data_saver.addAttribute('total meas num:',self.po2_x_positions.shape[0])  
#                 self.data_saver.addAttribute('x_pos',self.po2_x_positions[self.i_po2_point])            
#                 self.data_saver.addAttribute('y_pos',self.po2_y_positions[self.i_po2_point])              
#                 self.data_saver.addAttribute('depth',self.currentZPos)
#                 self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
#                 self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
#                 self.data_saver.addAttribute('last live scan:',self.liveScanNumber)
#                 self.data_saver.addAttribute('last stack acq:',self.stackNumber)
#                 self.data_saver.addAttribute('gate on [us]:',self.gate_on_2P_PO2)
#                 self.data_saver.addAttribute('gate off [us]:',self.gate_off_2P_PO2)
#                 self.data_saver.addAttribute('amplitude voltage ON:',self.voltage_on_2P_PO2)
#                 self.data_saver.addAttribute('freq acq. PO2 [Hz]:',self.freq_2P_PO2)
#                 self.data_saver.addAttribute('n averages:',self.n_averages_2P_PO2)
#                 self.data_saver.addAttribute('width',width)
#                 self.data_saver.addAttribute('height',height)                 
#                 self.data_saver.addAttribute('nx',nx_preview)
#                 self.data_saver.addAttribute('ny',ny_preview)         
#                 
#                 self.data_saver.setBlockSize(512)          
#                 self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
#                 self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
#                 self.data_saver.startSaving()
            #
            self.galvos.moveOnDemand(self.po2_x_positions[self.i_po2_point], self.po2_y_positions[self.i_po2_point])
            #self.toggle_shutter2ph()
            time.sleep(0.2)
            self.eom_task.writeOnce()
            self.eom_task.start()
        else:
            #self.shutter2ph.off() 
            #self.shutter2ph_closed = True
            #time.sleep(1)
            #self.pushButton_shutter_2ph.setStyleSheet("background-color: pale gray");
            #self.pushButton_shutter_2ph.setText('Shutter 2Ph: Closed')
            self.toggle_shutter2ph() 
            time.sleep(0.1)
            self.eom_task.close()
            self.galvos.ao_task.StopTask()
            self.galvos.ao_task.ClearTask()
            self.galvos.config()
            if self.checkBox_enable_save.isChecked():
                print('stop saving...')
                self.data_saver.stopSaving()
                self.ai_task.removeDataConsumer(self.data_saver)
                #self.data_saver.setBlockSize(512)
                print('stop saving done!')
            self.ai_task.setDecoder(self.galvos)
            #self.viewer.removeAllPoints()
            # Remettre EOM Manuel
            self.power_ao_eom = OnDemandVoltageOutTask(config.eom_device, config.eom_ao, 'Power2Ph')
            print('po2 acquisition done!')
            self.ai_task.updateConsumerFlag('viewer',True)        
            self.ai_task.removeDataConsumer(self.po2viewer)
            power2ph=self.horizontalScrollBar_power2ph.value()
            self.setPower2ph(power2ph)



    @pyqtSlot()       
    def next3PPO2Point(self):
        #self.shutter3ph.off() 
        #self.shutter3ph_closed = True
        #time.sleep(1)
        ##self.pushButton_shutter_3ph.setStyleSheet("background-color: pale gray");
        #self.pushButton_shutter_3ph.setText('Shutter 2Ph: Closed')        
        print('in next point!')
        time.sleep(0.5)
        self.i_po2_point = self.i_po2_point+1
        self.viewerPO2.highlightPointAll(self.i_po2_point)

        if(self.i_po2_point < self.po2_x_positions.shape[0]):
            #    Start acquisition task averaging doing a finite ao task reapeated n average times
            #    Show in a plot the Decay curve
            #   Should remove these prints once we know it works. No need to update viewer anymore
            print('PO2 measurement: ' + str(self.i_po2_point+1) + ' out of ' + str(self.po2_x_positions.shape[0]))
            print(str(self.po2_x_positions[self.i_po2_point])+';'+str(self.po2_y_positions[self.i_po2_point]))
           
#             if self.checkBox_enable_save.isChecked():
#                 if(self.i_po2_point>0):
#                     print('stop saving...')
#                     self.data_saver.stopSaving()
#                     self.ai_task.removeDataConsumer(self.data_saver)
#                 
#                 print('in save...')
#                 self.data_saver=DataSaver(self.save_filename)
#                 self.mouseName=self.lineEdit_mouse_name.text()
#                 self.scanType = '3P-po2plot'      
#                 self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
#                 self.po2PlotNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
#                 self.lineEdit_po2plot_acq.setText(str(self.po2PlotNumber))
#                 self.scanNumber = self.scanType+'_'+str(self.po2PlotNumber) 
#                 self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
#                 self.data_saver.setDatasetName(self.pathName) 
#                 self.data_saver.addAttribute('meas num:',self.i_po2_point+1)  
#                 self.data_saver.addAttribute('total meas num:',self.po2_x_positions.shape[0])  
#                 self.data_saver.addAttribute('x_pos',self.po2_x_positions[self.i_po2_point])            
#                 self.data_saver.addAttribute('y_pos',self.po2_y_positions[self.i_po2_point])              
#                 self.data_saver.addAttribute('depth',self.currentZPos)
#                 self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
#                 self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
#                 self.data_saver.addAttribute('last live scan:',self.liveScanNumber)
#                 self.data_saver.addAttribute('last stack acq:',self.stackNumber)
#                 self.data_saver.setBlockSize(512)          
#                 self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
#                 self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
#                 self.data_saver.startSaving()            

            self.galvos.moveOnDemand(self.po2_x_positions[self.i_po2_point], self.po2_y_positions[self.i_po2_point])
            time.sleep(0.5)
            #self.toggle_shutter3ph()
            self.galvo_gate_task.writeOnce()
            self.galvo_gate_task.start()
        else:
            
            #self.shutter3ph.off() 
            #self.shutter3ph_closed = True
            #time.sleep(1)
            #self.pushButton_shutter_3ph.setStyleSheet("background-color: pale gray");
            #self.pushButton_shutter_3ph.setText('Shutter 3Ph: Closed') 
            self.toggle_shutter3ph()
            time.sleep(0.2)
            
            self.setGalvoToPos3P(0)

            
            #self.galvo_gate_task.close()
            
            
            self.galvos.ao_task.StopTask()
            self.galvos.ao_task.ClearTask()
            self.galvos.config()

            if self.checkBox_enable_save.isChecked():
                print('stop saving...')
                self.data_saver.stopSaving()
                self.ai_task.removeDataConsumer(self.data_saver)
                #self.data_saver.setBlockSize(512)
                print('stop saving done!')
            self.ai_task.setDecoder(self.galvos)
            print('po2 acquisition done!')
            self.ai_task.updateConsumerFlag('viewer',True)        
            self.ai_task.removeDataConsumer(self.po2viewer)
           
            
    def stop_po2_scan(self):
        # Finish current point but stop after
        self.i_po2_point = self.po2_x_positions.shape[0]
        power2ph=self.horizontalScrollBar_power2ph.value()
        self.setPower2ph(power2ph) 

    def stop_3p_po2_scan(self):
        # Finish current point but stop after
        self.i_po2_point = self.po2_x_positions.shape[0]
        power3ph=self.horizontalScrollBar_power3ph.value()
        self.setPower3ph(power3ph) 
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
            self.checkBox_save_flag_enabled=self.checkBox_enable_save.isEnabled()
            self.checkBox_save_flag_checked=self.checkBox_enable_save.isChecked()
            self.checkBox_enable_save.setChecked(False)
            self.checkBox_enable_save.setEnabled(False)  
            self.previewFlag=1
        else: 
            self.lineEdit_nx.setText(self.nxTemp)
            self.lineEdit_ny.setText(self.nxTemp)
            self.lineEdit_linerate.setText(self.linerateTemp)            
            self.pushButton_preview.setText('Preview')
            self.checkBox_enable_save.setChecked(self.checkBox_save_flag_checked)
            self.checkBox_enable_save.setEnabled(self.checkBox_save_flag_enabled)
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
        self.po2viewer.clearPlot()



    #MULTIPLE LINE FUNCTIONS:
    
    def toggleDiameter(self):
        
        self.numberOfLines=len(self.viewer.lines)
        
        if self.numberOfLines == 0:
            print('no lines!')
            self.toggleDiameterFlag=True
            self.pushButton_addOrthogonalLines.setText('Add ortho.')            
        else:
        
            if self.toggleDiameterFlag:
                self.viewer.generateOrthogonalLines()
                self.updateNumberOfLines()
                self.toggleDiameterFlag=False
                self.diamFlag=True
                self.pushButton_addOrthogonalLines.setText('Remove ortho.')
            else:
                self.viewer.removeOrthogonalLines()
                self.updateNumberOfLines()
                self.toggleDiameterFlag=True
                self.diamFlag=False
                self.pushButton_addOrthogonalLines.setText('Add ortho.')



    def fixLengthLines(self):
        length=float(self.lineEdit_fixedLength.text())
        self.viewer.forceLength(length)
        
    def addLines(self):
        self.viewer.addLines()
        self.updateNumberOfLines()
        if self.numberOfLines>0:
            self.enableMoveButtons(True)

    def removeSelectedLine(self):
        self.viewer.removeSelectedLine()
        self.updateNumberOfLines()
        if self.numberOfLines<1:
            self.enableMoveButtons(False)
        
    def deleteAllLines(self):
        self.viewer.removeAllLines()
        if not self.toggleDiameterFlag:
            self.pushButton_addOrthogonalLines.setText('Add ortho.')
            self.toggleDiameterFlag=True            
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
        
    def updatePowerCurve(self,currentPosition,currentPowerValue):
        self.TwoPplot.clear()
        self.TwoPplot.plot(self.depthVector, self.powerCurve)
        #self.TwoPplot.setData(self.positionVector,self.powerVector)#,pen=None, symbol='o', symbolPen=None, symbolSize=4, symbolBrush=('r'))
        scatter = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='o', size=1)
        self.TwoPplot.addItem(scatter)
        scatter.setData(self.positionVector,self.powerVector)
        scatter2 = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='y'), symbol='o', size=1)
        self.TwoPplot.addItem(scatter2)
        scatter2.setData(np.array([currentPosition]),np.array([currentPowerValue]))        
        self.TwoPplot.show()
        
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
        nextra=int(self.lineEdit_extrapoints.text())
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())      
        
        height_forOCT=height*(nextra+ny)/ny
        
        
        linescan_x_1_um=linescan_px_x_1/nx*width
        linescan_x_2_um=linescan_px_x_2/nx*width
        linescan_y_1_um=linescan_px_y_1/(nextra+ny)*height
        linescan_y_2_um=linescan_px_y_2/(nextra+ny)*height

        linescan_shift_x=float(self.horizontalScrollBar_line_scan_shift_x.value())
        linescan_shift_y=float(self.horizontalScrollBar_line_scan_shift_y.value())

        linescan_x_1_um_forOCT=(linescan_px_x_1)/nx*width
        linescan_x_2_um_forOCT=(linescan_px_x_2)/nx*width
        linescan_y_1_um_forOCT=(linescan_px_y_1)/(ny+nextra)*height_forOCT
        linescan_y_2_um_forOCT=(linescan_px_y_2)/(ny+nextra)*height_forOCT
        
        center_x=(linescan_x_2_um+linescan_x_1_um)/2-width/2
        center_y=(linescan_y_2_um+linescan_y_1_um)/2-height/2
 
        center_x_forOCT=(linescan_x_1_um_forOCT+linescan_x_2_um_forOCT)/2-width/2
        center_y_forOCT=(linescan_y_2_um_forOCT+linescan_y_1_um_forOCT)/2-height_forOCT/2
        
        print('LINE: position: '+str(center_x)+','+str(center_y))
        
        linescan_width = math.sqrt(math.pow((linescan_x_2_um-linescan_x_1_um),2)+math.pow((linescan_y_2_um-linescan_y_1_um),2))
        
        strCoord=str(center_x_forOCT)+'/'+str(center_y_forOCT) +'/'+str(self.lineScanNumber+1)
            
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
        
    def shift_display_live_changed_value(self, value):
        print('in display value changed function')
        self.changedOffsetDisplayLive.emit(value)
        
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
            print ('Turning laser on...')
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
            print ('Turning laser off...')
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
            print ('Laser shutter open...')
            self.maitai.SHUTTERopen()
            self.maitaiShutter_closed = False
            self.pushButton_maitaiShutter.setStyleSheet("background-color: red")
            self.pushButton_maitaiShutter.setText('Maitai Shutter: OPEN')
        else:
            print ('Laser shutter closed...')
            self.maitai.SHUTTERshut() 
            self.maitaiShutter_closed = True
            self.pushButton_maitaiShutter.setStyleSheet("background-color: pale gray");
            self.pushButton_maitaiShutter.setText('Maitai Shutter: Closed')  
            
    def readPower_maitai(self):
        print ('Reading laser power...')
        powerMaitai = self.maitai.ReadPower()
        powerMaitai = str(round(float(powerMaitai[0:-2])*100.0)/100.0)
        self.lineEdit_laserResponse.setText("laser power: "+powerMaitai+"W")
        
    def checkPowerStatus_maitai(self):
        print ('Checking laser status...')
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
        print ('Laser diode current: ' + currentMaitai +' %')

    def readStatus_maitai(self):
        print ('Reading laser status...')
        statusMaitai = self.maitai.ReadStatus()
        self.lineEdit_laserResponse.setText("laser status: "+statusMaitai)   
        if(int(statusMaitai)==13):
            self.pushButton_maitaiML.setStyleSheet("background-color: red"); 
        elif(int(statusMaitai)==15):
            self.pushButton_maitaiML.setStyleSheet("background-color: green"); 
        else:
            self.pushButton_maitaiML.setStyleSheet("background-color: pale gray"); 
        
    def readWavelength_maitai(self):
        print ('Reading laser wavelength...')
        wavelengthMaitai = self.maitai.ReadWavelength()
        self.lineEdit_laserResponse.setText("laser wavelength: " + wavelengthMaitai) 
        
    def setWavelength_maitai(self):
        wavelength_command = self.lineEdit_wavelength.text()
        print ('Setting laser wavelength... ' + wavelength_command +' nm')
        wavelengthMaitai=self.maitai.ChangeWavelength(int(wavelength_command))
        print (wavelengthMaitai)
        while (wavelengthMaitai[0:-3]!=wavelength_command):
            wavelengthMaitai = self.maitai.ReadWavelength()
        print ('wavelength set!')
        
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
        self.update2Ppower()

        if not(self.simultaneousPO2LS):
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
            #self.update3Ppower()
            

            
            
    def wheel3ph_plus5(self):
        value=self.horizontalScrollBar_power3ph.value()+5
        if value>100:
            value=100
        self.horizontalScrollBar_power3ph.setValue(value)
            
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        
    def wheel3ph_minus5(self):
        value=self.horizontalScrollBar_power3ph.value()-5
        if value<1:
            value=1
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        self.horizontalScrollBar_power3ph.setValue(value)

    def wheel3ph_plus14(self):
        value=self.horizontalScrollBar_power3ph.value()+14
        self.horizontalScrollBar_power3ph.setValue(value)
        if value>100:
            value=100
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        self.horizontalScrollBar_power3ph.setValue(value)
        
    def wheel3ph_gotomax(self):
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        self.horizontalScrollBar_power3ph.setValue(100)
        
    def wheel3ph_gotomin(self):
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        self.horizontalScrollBar_power3ph.setValue(1)
        
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
        print ("Starting rotation:")
        self.thorlabs.setpos(command)
        self.flagWheel = 1
        print ("Wheel rotation done!")
        
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
        print (self.lineEdit_laser_power1.text())
        self.power_motor.goTo_abs(1, deg)
        
    def set_power2(self): 
        percent = float(self.lineEdit_laser_power2.text())
        #calib = 7*math.pi/45
        calib = 40*math.pi/45
        deg=float((45/math.pi)*(math.asin(2*percent/100-1)+math.pi/2+calib))
        print (self.lineEdit_laser_power2.text())
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
        print ('set_brain_pos: in function')
        self.brain_pos=self.motors.get_pos(3)
        print(self.brain_pos)
        print ('set_brain_pos: done')
        self.pushButton_goto_brain.setEnabled(True)    
        self.lineEdit_brain_pos.setText(self.brain_pos)
        self.brainPosSetFlag = 0
        self.pushButton_start_stack.setText('Set brain pos. first!')
        
    def set_brain_pos(self):    
        self.set_xy_center()
        print ('set_brain_pos: in function')
        self.brain_pos=self.motors.get_pos(3)
        print(self.brain_pos)
        print ('set_brain_pos: done')
        self.pushButton_goto_brain.setEnabled(True)    
        self.lineEdit_brain_pos.setText(self.brain_pos)
        self.brainPosSetFlag = 1
        self.pushButton_add_power_val.setEnabled(True)
        self.pushButton_add_power_val_3P.setEnabled(True)
        self.pushButton_start_stack.setEnabled(True)
        self.pushButton_start_stack.setText('Start Stack')
        self.pushButton_start_linescan.setEnabled(True)
        self.pushButtonStartMultipleLineAcq.setEnabled(True)       
        self.pushButton_startPO2.setEnabled(True)
        self.pushButton_start_3p_po2_scan.setEnabled(True)
        self.pushButton_stopPO2.setEnabled(True)
        self.pushButton_stop_3p_po2_scan.setEnabled(True)       
        self.pushButton_stop_linescan.setEnabled(True)
        self.pushButtonStopMultipleLineAcq.setEnabled(True)  
        self.get_current_xy_position()
        self.pushButton_powercurve_start.setEnabled(True)
        self.pushButton_powercurve_stop.setEnabled(False)
              
        
    def motors_home(self):
        self.motors.home()
        
    def get_current_z_depth(self):
        if self.brainPosSetFlag==1:
            self.abs_pos=self.motors.get_pos(3)
            self.currentZPos=float(self.brain_pos)-float(self.abs_pos)
            self.lineEdit_currentZpos.setText(str(self.currentZPos))
        else:
            print ('brain position not set yet!')
                
    def get_current_xy_position(self):
        if self.brainPosSetFlag==1:
            self.abs_pos_x=self.motors.get_pos(1)
            self.abs_pos_y=self.motors.get_pos(2)
            self.currentXPos=float(self.center_x_FOV)-float(self.abs_pos_x)
            self.currentYPos=float(self.center_y_FOV)-float(self.abs_pos_y)
            txt=str(int(self.currentXPos*1000.0))+'/'+str(int(self.currentYPos*1000.0))
            self.lineEdit_xy_position.setText(txt)
        else:
            print ('brain position not set yet!')
                
    def set_xy_center(self):       
        print ('set_x_center: in function')
        self.center_x_FOV=self.motors.get_pos(1)
        print(self.center_x_FOV)
        print ('set_x_center: done')
        print ('set_y_center: in function')
        self.center_y_FOV=self.motors.get_pos(2)
        print(self.center_y_FOV)
        print ('set_y_center: done'  )  
        self.pushButton_center.setEnabled(True)    
           
    def goto_brain_pos(self):       
        print ('goto_brain_pos: in function')
        #self.actual_pos=self.motors.get_pos_axial()
        #commandPos=float(self.brain_pos)-float(self.actual_pos)
        self.motors.move_az(self.brain_pos)
        self.get_current_z_depth()
        print ('goto_brain_pos: done' )   
        


#FUNCTIONS SCANNING:
        
    def startlinescanTimer(self):
        self.currentLine = 0
        self.timerLS = pg.QtCore.QTimer()
        self.timerLS.timeout.connect(self.stoplinescanTimer)

        self.make_connection_offset_X()
        self.make_connection_offset_Y()
        self.make_connection_offset_Display()
        shift_display=self.horizontalScrollBar_line_scan_shift_display.value()
        self.viewer.move_offset_display(shift_display)
        self.viewer2.move_offset_display(shift_display)  
        print("creating current line viewer...")
        print("creating current line viewer...done!")      
        angio=self.viewer.getCurrentImage()
        self.currentViewer=CurrentLineViewer(angio)
        self.runlinescanTimer()
        self.hideLines()

             
     
    def is_odd(self,num):
        return (num % 2)>0
        
    def runlinescanTimer(self):
        timeOffset=1.1 #offset of seconds measured   
        self.timestep=(float(self.lineEdit_TimePerLine.text())+timeOffset)*1000
        linescan_type='parallel'
        
        if self.diamFlag:
            if self.is_odd(self.currentLine):
                self.timestep=(float(self.lineEdit_TimePerLinePerp.text())+timeOffset)*1000
                print('current line is perpendicular...')
                linescan_type='perpendicular'
            else:
                self.timestep=(float(self.lineEdit_TimePerLine.text())+timeOffset)*1000
                print('current line is parallel...')
                linescan_type='parallel'
        
        print('timestep of:' + str(int(self.timestep)))
                
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
        x_currentLineCenter,y_currentLineCenter,angle_currentLine,length_currentLine=self.viewer.getAngleAndCenterSelectedLinePosition(self.currentLine)
        self.currentViewer.displayCurrentLine(x_currentLineCenter,y_currentLineCenter,angle_currentLine,length_currentLine)

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
            self.data_saver.addAttribute('linescan_type',linescan_type)
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
            self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
            self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
            self.data_saver.addAttribute('last_live_scan',self.liveScanNumber)
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

    def updateTimer(self):
        self.linescantimercounter+=1
        #print("timer time is: "+str(self.linescantimercounter/10))
        self.lcdNumber_timer.display(str(float(self.linescantimercounter/10)))

    def run_timer(self):
        self.lcdNumber_timer.setDigitCount(5)
        self.linescantimercounter=0
        self.linescantimer = pg.QtCore.QTimer()
        self.linescantimer.timeout.connect(self.updateTimer)
        self.linescantimer.start(100)      
    
    def stoptimer(self):
        self.linescantimer.stop()              
        
    def startlinescan(self):
        
        self.update_linescan()
        if (self.simultaneousPO2LS or self.eomLSFlag):
            print('making connection for PO2-LS')
            self.toggle_shutter2ph()
            self.power_ao_eom.write(0)
            
            self.make_connection_power_center_illumination()
            self.make_connection_power_short_illumination()
            self.make_connection_pos_center_illumination()
            self.make_connection_change_center_illumination()
            self.make_connection_move_short_illumination()
            self.make_connection_change_short_illumination()
            self.make_connection_power_short_illumination()   
            self.n_lines_previous=int(self.lineEdit_nt.text())
            self.nx_preview_previous=int(self.lineEdit_nx.text())
            self.ny_preview_previous=int(self.lineEdit_ny.text())
            self.n_extra_previous=int(self.lineEdit_extrapoints_LS.text())   
            self.npts_previous=int(self.lineEdit_nr.text())  
            if self.simultaneousPO2LS:
                powerLS, powerPO2, onTimeRatioCenter, onTimePositionCenter, onTimePositionShort, on_time =  self.updateParametersSimultaneous_PO2LS()
            elif self.eomLSFlag:
                powerLS, powerPO2, onTimeRatioCenter, onTimePositionCenter, onTimePositionShort, on_time =  self.updateParameters_EOM_LS()
                
            
            
            try:
                self.power_ao_eom.ClearTask()
            except AttributeError:
                print('eom already cleared!')
            else:
                del(self.power_ao_eom)
        else:
            self.setInitialPower()
        
        x_0_px, y_0_px, x_e_px, y_e_px, linescan_length_px = self.viewer.getMouseSelectedLinePosition()
        self.viewer.setLineScanFlag(True)
        self.viewer2.setLineScanFlag(True)
        if (x_0_px==0 and y_0_px==0 and x_e_px==0 and y_e_px==0):
            print("no lines selected!")
        else:
            if not(self.simultaneousPO2LS or self.eomLSFlag):
                if self.checkBoxLive2P.isChecked():
                    self.toggle_shutter2ph()
                if self.checkBoxLive3P.isChecked():
                    self.toggle_shutter3ph()
            self.viewer.toggleLinearSelection()
            self.make_connection_offset_X()
            self.make_connection_offset_Y()
            self.make_connection_offset_Display()
            self.get_current_z_depth()
        
            if self.previewScanFlag == True:
                self.stopscan()
                if self.checkBoxLive2P.isChecked():
                    self.toggle_shutter2ph()
                if self.checkBoxLive3P.isChecked():
                    self.toggle_shutter3ph()
        
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
            
            if (self.simultaneousPO2LS or self.eomLSFlag):
                print('creating ramp PO2LS')
                [npts,n_extra,n_lines]=self.galvos.setLineRamp(y_0_um,x_0_um,y_e_um,x_e_um,npts,n_lines,n_extra,line_rate,shift_display)
            else:
                self.galvos.setLineRamp(y_0_um,x_0_um,y_e_um,x_e_um,npts,n_lines,n_extra,line_rate,shift_display)
                
            self.lineEdit_nr.setText(str(npts))
            self.lineEdit_nt.setText(str(n_lines))
            self.lineEdit_extrapoints_LS.setText(str(n_extra))
            
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
                self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
                self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
                self.data_saver.addAttribute('last_live_scan',self.liveScanNumber)
                self.data_saver.addAttribute('shutter_open_2P',self.checkBoxLive2P.isChecked())
                self.data_saver.addAttribute('shutter_open_3P',self.checkBoxLive3P.isChecked())
                now=datetime.now()
                currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
                self.data_saver.addAttribute('acquisition_time',currentdate)
                if (self.simultaneousPO2LS or self.eomLSFlag):
                    self.data_saver.addAttribute('powerLS',powerLS)
                    self.data_saver.addAttribute('powerPO2',powerPO2)
                    self.data_saver.addAttribute('onTimeRatioCenter',onTimeRatioCenter)
                    self.data_saver.addAttribute('onTimePositionCenter',onTimePositionCenter)
                    self.data_saver.addAttribute('onTimePositionShort',onTimePositionShort)
                    self.data_saver.addAttribute('on_time',on_time)
                else:
                    self.data_saver.addAttribute('power2P',self.horizontalScrollBar_power2ph.value())
                    self.data_saver.addAttribute('power3P',self.horizontalScrollBar_power3ph.value())                    
                self.data_saver.setBlockSize(512)          
                self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
                self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
                self.data_saver.startSaving()
            if self.checkBox_activateTimer.isChecked():
                print("starting timer process...")
                self.run_timer()
                print("starting timer process...done!")
            self.galvos.startTask()


    def stoplinescan(self):
        
        self.shutter2ph_closed = False
        self.toggle_shutter2ph()
        self.shutter3ph_closed = False
        self.toggle_shutter3ph()
        self.viewer.toggleLinearSelection()
        if self.checkBox_activateTimer.isChecked():
            print("ending timer process...")
            self.stoptimer()
            print("ending timer process...done!")
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
        if (self.simultaneousPO2LS or self.eomLSFlag):
            self.power_ao_eom = OnDemandVoltageOutTask(config.eom_device, config.eom_ao, 'Power2Ph')
            self.lineEdit_nt.setText(str(self.n_lines_previous))
            self.lineEdit_nx.setText(str(self.nx_preview_previous))
            self.lineEdit_ny.setText(str(self.ny_preview_previous))
            self.lineEdit_extrapoints_LS.setText(str(self.n_extra_previous))
            self.lineEdit_nr.setText(str(self.npts_previous))
            self.setInitialPower()        
        self.simultaneousPO2LS=0
        self.checkBox_activatePO2LS.setChecked(0)

        self.toggle_PO2_LS()
        self.power_ao_eom.write(0)
            
        
    def toggle_averaging(self):
        av=int(self.lineEdit_averagingRange.text())
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text()) 
        if self.checkBox_average.isChecked():
            print('Averaging On!')
            self.viewer.checkAverageFlag(True,av,nx+n_extra,ny)
            self.viewer2.checkAverageFlag(True,av,nx+n_extra,ny)
        else:
            print('Averaging Off!')
            self.viewer.checkAverageFlag(False,av,nx+n_extra,ny)
            self.viewer2.checkAverageFlag(False,av,nx+n_extra,ny)   
            
    def disable_averaging(self):
        print('Averaging Off!')
        av=int(self.lineEdit_averagingRange.text())
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text())
        self.viewer.checkAverageFlag(False,av,nx+n_extra,ny)
        self.viewer2.checkAverageFlag(False,av,nx+n_extra,ny)      
        self.checkBox_average.setChecked(False)
        
    def update2Ppower(self):
        self.powerValue2P=int(self.horizontalScrollBar_power2ph.value())
        if self.powerValue2P>99:
            self.powerValue2P=99
        self.powerValue2P=np.floor(self.powerValues2P[self.powerValue2P]/10)*10
        txt=str(self.powerValue2P)+' mW'
        self.label_powerValue2p.setText(txt)
        
    def update3Ppower(self):
        self.powerValue3P=int(self.horizontalScrollBar_power3ph.value())
        self.powerValue3P=np.floor(self.powerValues3P[self.powerValue3P]/10)*10
        txt=str(self.powerValue3P)+' mW'
        self.label_powerValue3p.setText(txt)      
        
    def startscan(self):
        self.update2Ppower()
        #if self.checkBoxLive3P.isChecked():
            #self.update3Ppower()        
        self.checkBox_activateEOM.setChecked(0)
        self.eomLSFlag=0
        self.setInitialPower()
        self.make_connection_offset_Display_live()
        self.viewer.getScanningType(self.comboBox_scantype.currentText())
        self.viewer2.getScanningType(self.comboBox_scantype.currentText())
        self.intensityFlag=self.checkBox_trackIntensity.isChecked()
        self.viewer.plotIntensityFlag(self.intensityFlag)
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
        if self.brainPosSetFlag:
            self.get_current_z_depth()
            currentdepth=self.currentZPos
        else:
            currentdepth=0
        print('current depth: '+str(currentdepth))
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
            self.data_saver.addAttribute('depth',currentdepth)
            self.comment=(self.lineEdit_comment.text())
            self.data_saver.addAttribute('comment',self.comment)
            if not(self.shutter2ph_closed):
                self.data_saver.addAttribute('laser','MaiTai')
                self.data_saver.addAttribute('power2P',self.powerValue2P)
                val=self.horizontalScrollBar_power2ph.value()
                self.data_saver.addAttribute('2Pperc',val)
            elif not(self.shutter3ph_closed):
                self.data_saver.addAttribute('laser','3P Soliton')
                #self.data_saver.addAttribute('power3P',self.powerValue3P)
                val=self.horizontalScrollBar_power3ph.value()
                self.data_saver.addAttribute('3Pperc',val)
            else:
                self.data_saver.addAttribute('laser','None')
                
               
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
        self.disable_averaging()


    def stack_thread(self):
        
        self.disable_averaging()

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
            print (k)
                       
            print('scanning started')   
            self.make_connection(self.galvos.ao_task.signal_helper.aoDoneSignal)         
            self.galvos.startTask()    
        
            print ('scan started')
            self.motors.move_dz(-self.zstep/1000)
            
            print ('motor moved')
            if self.stopped:
                break
        self.goto_brain_pos()

        #time.sleep(2)
        
        self.galvos.setFinite(False)
    
    def take_snapshot(self):
        self.disable_averaging()
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
        self.disable_averaging()
        self.checkBox_activateEOM.setChecked(0)
        self.eomLSFlag=0
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
            print ('setting saving')
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = 'snapshot'      
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.stackNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            print ('data checked')
            self.lineScanNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            self.lineEdit_linescan_acq.setText(str(self.lineScanNumber))
            self.scanNumber = self.scanType+'_'+str(self.lineScanNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            print ('setting name:')
            print (self.pathName )          
            self.data_saver.setDatasetName(self.pathName)       
            self.data_saver.addAttribute('nx',nx)
            self.data_saver.addAttribute('ny',ny)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.setBlockSize(512)
            print ('setting consumers')
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            print ('starting saving')
            self.data_saver.startSaving()
        #        print 'making connection'
        self.make_connection_snapshot(self.galvos.ao_task.signal_helper.aoDoneSignal)
        print ('starting task')
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
        self.setInitialPower()
        self.disable_averaging()
        
        
        self.viewer.setLineScanFlag(True)
        self.viewer2.setLineScanFlag(True)
        self.pushButtonStartMultipleLineAcq.setEnabled(False)
        self.pushButtonStopMultipleLineAcq.setEnabled(True)
        print('in START MULTIPLE LINES')  
        
        
        self.currentViewer=CurrentLineViewer(angio)
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

        x_currentLineCenter,y_currentLineCenter,angle_currentLine,length_currentLine=self.viewer.getAngleAndCenterSelectedLinePosition(self.currentLine)
        self.currentViewer.displayCurrentLine(x_currentLineCenter,y_currentLineCenter,angle_currentLine,length_currentLine)


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
            self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
            self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
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

        x_currentLineCenter,y_currentLineCenter,angle_currentLine,length_currentLine=self.viewer.getAngleAndCenterSelectedLinePosition(self.currentLine)
        self.currentViewer.displayCurrentLine(x_currentLineCenter,y_currentLineCenter,angle_currentLine,length_currentLine)
            
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
                    self.data_saver.addAttribute('x_FOV_center',self.currentXPos)
                    self.data_saver.addAttribute('y_FOV_center',self.currentYPos)
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
        self.checkBox_activateEOM.setChecked(0)
        self.eomLSFlag=0
        self.setInitialPower()
        self.make_connection_offset_Display_live()
        self.viewer.getScanningType(self.comboBox_scantype.currentText())
        self.viewer2.getScanningType(self.comboBox_scantype.currentText())
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
        self.currentPositionStack=0

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
            print ('setting saving')
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = ('Stack')   
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.stackNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            print ('data checked')
            self.lineEdit_stack_acq.setText(str(self.stackNumber))
            self.scanNumber = self.scanType+'_'+str(self.stackNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            print ('setting name:')
            print (self.pathName)         
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
            print ('setting consumers')
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            print ('starting saving')
            self.get_current_z_depth()
            self.lineEdit_current_depth.setText(str(self.currentZPos))
            self.lineEdit_current_depth_3P.setText(str(self.currentZPos))

            self.data_saver.startSaving()
        #
        self.set_iteration_number(0)
        print ('making connection')
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
        
        print ('starting task')                 
        self.galvos.startTask()    
    
    def stop_stack(self):
        self.galvos_stopped=True        

    def make_connection(self, slider_object):
        slider_object.connect(self.nextAcq)
        
    @pyqtSlot()
    def nextAcq(self):
        print('in nextAcq')
        now = datetime.now()
        currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
        print("start: ", currentdate)
        self.iteration_number=self.get_iteration_number()
        self.averagingVal = int(self.lineEdit_averaging.text())
        
        progVal=self.iteration_number*100/(self.n_steps*self.averagingVal)+1
        self.progressBar_stack.setValue(progVal)        
        self.galvos.stopTask()    
        self.set_iteration_number(self.iteration_number+1) 
         
        self.lineEditSliceUnderAcq.setText(str(self.iteration_number/self.averagingVal))  
        self.lineEdit_AvgNumber.setText(str(((self.iteration_number) % self.averagingVal)+1))
        
        stepVal=(self.n_steps*self.averagingVal)-1
        iterationCond=(self.iteration_number < stepVal)
        galvo_on=not(self.galvos_stopped)
        if (iterationCond & galvo_on):
            if ((((self.iteration_number) % self.averagingVal)==0) & (self.iteration_number>0)):
                print('--- moving motors')
                self.motors.move_dz(-self.zstep/1000)
                print('--- done!')
                self.currentZPos=self.currentZPos+self.zstep/1000
                self.powerIt=self.powerIt+1;
                self.currentPositionStack=self.currentPositionStack+self.zstep
                self.lineEdit_currentZpos.setText(str(self.currentPositionStack/1000))
                print('--- power number: ' + str(self.powerIt))
                if self.checkBoxPowerCurve.isChecked():
                    currentPowerValue=self.powerCurve[self.powerIt]
                    print('2P: power value: ' + str(round(currentPowerValue,1)) + '%')
                    if self.checkBoxStack2P.isChecked():
                        self.setPower2ph(currentPowerValue)
                        self.horizontalScrollBar_power2ph.setValue(currentPowerValue)
                        txt= str(round(currentPowerValue,1)) + '%'
                        self.lineEdit_current_power.setText(txt)
                        self.updatePowerCurve(self.currentZPos,round(currentPowerValue,1))
                if self.checkBoxPowerCurve3P.isChecked():
                    currentPowerValue3P=self.powerCurve3P[self.powerIt]
                    print('3P: power value: ' + str(round(currentPowerValue3P,1)) + '%')
                    if self.checkBoxStack3P.isChecked():                    
                        self.setPower3ph_noThread(currentPowerValue3P)
                        self.horizontalScrollBar_power3ph.setValue(currentPowerValue3P)
                        txt= str(round(currentPowerValue3P,1)) + '%'
                        self.lineEdit_current_power_3P.setText(txt)
            #time.sleep(2)
            now = datetime.now()
            currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
            print("step 1: ", currentdate)
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
            self.lineEdit_current_depth.setText(str(self.currentZPos))
            self.lineEdit_current_depth_3P.setText(str(self.currentZPos))
            self.make_connection(self.galvos.ao_task.signal_helper.aoDoneSignal) 
            now = datetime.now()
            currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
            print("step 2: ", currentdate)  
            self.galvos.startTask()
            now = datetime.now()
            currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
            print("step 3: ", currentdate)
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


#POWER CURVE ACQUISITON:
    def start_power_curve_acquisition(self):
        self.pushButton_powercurve_stop.setEnabled(True)

        self.checkBox_activateEOM.setChecked(0)
        self.eomLSFlag=0
        self.setInitialPower()
        self.make_connection_offset_Display_live()
        self.viewer.getScanningType(self.comboBox_scantype.currentText())
        self.viewer2.getScanningType(self.comboBox_scantype.currentText())

        self.pushButton_powercurve_start.setEnabled(False)
        self.pushButton_powercurve_stop.setEnabled(True)
        
        self.shutter2ph_closed=self.checkBox_powercurve_2P.isChecked()
        self.shutter3ph_closed=self.checkBox_powercurve_3P.isChecked()
        self.toggle_shutter2ph()
        self.toggle_shutter3ph()
        self.checkBox_powercurve_2P.setEnabled(False)
        self.checkBox_powercurve_3P.setEnabled(False)
        
        self.galvos.setFinite(True)
        self.goto_brain_pos()
        self.progressBar_stack.setValue(0)
        nx=int(self.lineEdit_nx.text())    
        ny=int(self.lineEdit_ny.text())   
        n_extra=int(self.lineEdit_extrapoints.text())                     
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())         
        line_rate=float(self.lineEdit_linerate.text())
        time.sleep(1)
        # Step two: Start stack loop
        startPower=float(self.lineEdit_powercurve_startpower.text())
        finalPower=float(self.lineEdit_powercurve_finalpower.text())
        stepPower=float(self.lineEdit_powercurve_powerstep.text())
        self.powercurve_averaging=int(self.lineEdit_powercurve_averaging.text())
        self.powerCurve=np.arange(startPower,finalPower+1,stepPower)
        self.powerCurve=np.repeat(self.powerCurve,self.powercurve_averaging)
        self.numberOfMeasurements = len(self.powerCurve)
        self.currentPositionStack=0
        self.lineEdit_powercurve_numberofmeas.setText(str(self.numberOfMeasurements))
        # Take 2D image and saving
        # Set ramp
        self.powerIt=-1
        if self.comboBox_scantype.currentText() == 'SawTooth':
            self.galvos.setSawToothRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
        elif self.comboBox_scantype.currentText() == 'Triangular':
            self.galvos.setTriangularRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,1,line_rate)
        elif self.comboBox_scantype.currentText() == 'Line':          
            self.galvos.setLineRamp(self.center_x-width/2,self.center_y-height/2,self.center_x+width/2,self.center_y+height/2,nx,ny,n_extra,line_rate)
        self.galvos_stopped = False
        #SET SAVING PART:
        if self.checkBox_enable_save.isChecked():
            print ('setting saving')
            self.data_saver=DataSaver(self.save_filename)
            self.mouseName=self.lineEdit_mouse_name.text()
            self.scanType = ('PowerCurve')   
            self.pathRoot = posixpath.join('/',self.mouseName,self.scanDate,self.scanType)
            
            self.powercurveNumber = self.data_saver.checkAlreadyExistingFiles(self.pathRoot,self.scanType)
            print ('data checked')
            self.lineEdit_stack_acq.setText(str(self.powercurveNumber))
            self.scanNumber = self.scanType+'_'+str(self.powercurveNumber) 
            self.pathName = posixpath.join(self.pathRoot,self.scanNumber)
            print ('setting name:')
            print (self.pathName)         
            self.data_saver.setDatasetName(self.pathName)       
            self.data_saver.addAttribute('nx',nx)
            self.data_saver.addAttribute('ny',ny)
            self.data_saver.addAttribute('n_extra',n_extra)
            self.data_saver.addAttribute('width',width)
            self.data_saver.addAttribute('height',height)            
            self.data_saver.addAttribute('line_rate',line_rate)   
            self.data_saver.addAttribute('scantype',self.comboBox_scantype.currentText())  
            self.data_saver.addAttribute('averaging',int(self.powercurve_averaging))  
            self.data_saver.addAttribute('powervalues',self.powerCurve) 
            self.data_saver.setBlockSize(512)
            print ('setting consumers')
            self.ai_task.setDataConsumer(self.data_saver,True,0,'save',True)
            self.ai_task.setDataConsumer(self.data_saver,True,1,'save',True)
            print ('starting saving')
            self.get_current_z_depth()
            self.lineEdit_current_depth.setText(str(self.currentZPos))
            self.lineEdit_current_depth_3P.setText(str(self.currentZPos))

            self.data_saver.startSaving()
        #
        self.set_iteration_number(0)
        print ('making connection')
        self.make_connection_powercurve(self.galvos.ao_task.signal_helper.aoDoneSignal)
        
        currentPowerValue=self.powerCurve[0]
        if self.checkBox_powercurve_2P.isChecked():
            self.setPower2ph(currentPowerValue)
            self.horizontalScrollBar_power2ph.setValue(currentPowerValue)
            txt= str(round(currentPowerValue,1)) + '%'
            self.lineEdit_current_power.setText(txt)

        if self.checkBox_powercurve_3P.isChecked():
            self.setPower3ph_noThread(currentPowerValue)
            self.horizontalScrollBar_power3ph.setValue(currentPowerValue)
            txt= str(round(currentPowerValue,1)) + '%'
            self.lineEdit_current_power_3P.setText(txt)
        
        print ('starting task')                 
        self.galvos.startTask()  
        
    def stop_power_curve_acquisition(self):
        self.galvos_stopped=True        

    def make_connection_powercurve(self, slider_object):
        slider_object.connect(self.nextAcqPowerCharac)        

    @pyqtSlot()
    def nextAcqPowerCharac(self):
        print('in nextAcqPowerCharac')
        now = datetime.now()
        currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
        print("start: ", currentdate)
        self.iteration_number=self.get_iteration_number()
        self.averagingVal = int(self.lineEdit_averaging.text())
        
        progVal=self.iteration_number*100/(self.numberOfMeasurements)+1
        self.progressBar_powercurve.setValue(progVal)        
        self.galvos.stopTask()    
        self.set_iteration_number(self.iteration_number+1) 
         
        self.lineEditSliceUnderAcq.setText(str(self.iteration_number/self.averagingVal))  
        self.lineEdit_AvgNumber.setText(str(((self.iteration_number) % self.averagingVal)+1))
        
        galvo_on=not(self.galvos_stopped)
        print('numberOfMeasurements '+str(self.numberOfMeasurements))
        print('powerIt '+str(self.powerIt))
        
        iterationCond=(self.powerIt < self.numberOfMeasurements-1)
        if (iterationCond & galvo_on):        
            self.powerIt+=1      
            print('--- power number: ' + str(self.powerIt))
            if self.checkBox_powercurve_2P.isChecked():
                currentPowerValue=self.powerCurve[self.powerIt]
                print('2P: power value: ' + str(round(currentPowerValue,1)) + '%')
                self.setPower2ph(currentPowerValue)
                self.horizontalScrollBar_power2ph.setValue(currentPowerValue)
                txt= str(round(currentPowerValue,1)) + '%'
                self.lineEdit_current_power.setText(txt)
            if self.checkBox_powercurve_3P.isChecked():
                currentPowerValue3P=self.powerCurve[self.powerIt]
                print('3P: power value: ' + str(round(currentPowerValue3P,1)) + '%')
                self.setPower3ph_noThread(currentPowerValue3P)
                self.horizontalScrollBar_power3ph.setValue(currentPowerValue3P)
                txt= str(round(currentPowerValue3P,1)) + '%'
                self.lineEdit_current_power_3P.setText(txt)
            time.sleep(0.5)
            now = datetime.now()
            currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
            print("step 1: ", currentdate)
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
            self.make_connection_powercurve(self.galvos.ao_task.signal_helper.aoDoneSignal)   
            now = datetime.now()
            currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
            print("step 2: ", currentdate)
            self.galvos.startTask()
            now = datetime.now()
            currentdate=now.strftime("%m/%d/%Y, %H:%M:%S")
            print("step 3: ", currentdate)
        else:
            print('acquisition finished!')
            self.shutter2ph_closed=False
            self.shutter3ph_closed=False  
            self.toggle_shutter2ph()
            self.toggle_shutter3ph()
            self.checkBox_powercurve_2P.setEnabled(True)
            self.checkBox_powercurve_3P.setEnabled(True)   
            self.galvos.setFinite(False)
            self.progressBar_stack.setValue(0)
            self.pushButton_powercurve_start.setEnabled(True)
            self.pushButton_powercurve_stop.setEnabled(False)
            if self.checkBox_powercurve_2P.isChecked():
                self.setPower2ph(0)
                self.horizontalScrollBar_power2ph.setValue(0)
            if self.checkBox_powercurve_3P.isChecked():
                self.setPower3ph(0)
                self.horizontalScrollBar_power3ph.setValue(0)
            if self.checkBox_enable_save.isChecked():
                print('stop saving...')
                self.data_saver.stopSaving()
                self.ai_task.removeDataConsumer(self.data_saver)
                print('stop saving done!')

    def make_connection_power_short_illumination(self):
        self.horizontalScrollBar_LSPO2Power.valueChanged.connect(self.galvos.change_power_short_illumination)

    def make_connection_power_center_illumination(self):
        self.horizontalScrollBar_power2ph.valueChanged.connect(self.galvos.change_power_center_illumination)

    def make_connection_pos_center_illumination(self):
        self.horizontalScrollBar_move_center_illumination.valueChanged.connect(self.galvos.move_position_center_illumination)

    def make_connection_change_center_illumination(self):
        self.horizontalScrollBar_change_center_illumination.valueChanged.connect(self.galvos.change_duration_center_illumination)        

    def make_connection_move_short_illumination(self):
        self.horizontalScrollBar_move_short_illumination.valueChanged.connect(self.galvos.move_position_short_illumination)

    def make_connection_change_short_illumination(self):
        self.horizontalScrollBar_change_short_illumination.valueChanged.connect(self.galvos.change_duration_short_illumination)  
        
    def make_connection_offset_X(self):
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.galvos.move_offset_X)        

    def make_connection_offset_Y(self):
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.galvos.move_offset_Y)        
        
    def make_connection_offset_Display(self):
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.viewer.move_offset_display)        
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.viewer2.move_offset_display)        
    
    def make_connection_offset_Display_live(self):
        self.horizontalScrollBar_shift.valueChanged.connect(self.viewer.move_offset_display_live)        
        self.horizontalScrollBar_shift.valueChanged.connect(self.viewer2.move_offset_display_live) 
    
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

    def motors_walk(self):
        print('start walk...')
        self.distance_random_walk=float(self.lineEdit_walkamplitude.text())
        self.timer_motor_walk = pg.QtCore.QTimer()
        self.time=float(self.lineEdit_timeperwalk.text())
        
        oldspeed=self.motors.get_velocity_x()
        self.oldspeed=float(oldspeed[:-2])
        walkintialposition=self.motors.get_pos(1)
        
        self.walkintialposition=float(walkintialposition[:-2])
        
        newspeed=self.distance_random_walk/self.time
        
        self.motors.set_velocity_x(newspeed)
        
        newvel=self.motors.get_velocity_x()
        
        print('motor speed: '+str(newspeed))
        print('motor speed actual: '+str(newvel))
        
        print('amplitude: '+str(self.distance_random_walk))
        print('time: '+str(self.time))
        

        self.timer_motor_walk.timeout.connect(self.motors_walk_thread)
        self.timer_motor_walk_flag=1
        self.timer_motor_walk.start(self.time*1000)
        #self.thread_motor_walk = threading.Thread(target=self.motors_walk_thread)
        #self.thread_motor_walk.start()

        

    def stop_motors_walk_thread(self):   
        print('stop walk...')     
        if self.timer_motor_walk_flag:
            self.timer_motor_walk.stop()
        print('finished walk...')     
        self.motors.set_velocity_x(self.oldspeed)

    def motors_walk_thread(self):
        self.distance_random_walk=-self.distance_random_walk
        cmd=self.walkintialposition+self.distance_random_walk
        print('in loop: distance '+str(cmd))
        self.motors.move_ax(cmd)

            
        
        #cmd_velocity=float(self.old_velocity[:-2])
        #self.motors.set_velocity_x(cmd_velocity*2)
        #self.new_velocity=self.motors.get_velocity_x()
        #print('velocity' + self.new_velocity)
        
        #while self.randomwalkOn:
        #    print('in loop')
        
        
        #self.motors.set_velocity_x(cmd_velocity)
        #self.new_velocity=self.motors.get_velocity_x()
        #print('velocity' + self.new_velocity)            
        #while self.motorswalk:

        
        
    def stopOscillatePiezoTimer(self):
        if self.timerPiezoFlag:
            self.timerPiezo.stop()
            self.stopPiezo()
        
    def oscillatePiezo(self):
        print('piezo oscillate')
        speed=self.spinBox_piezo.value()
        time=int(self.lineEdit_piezo_time_per_iter.text())
        num=int(self.lineEdit_piezo_number_iter.text())
        self.piezo.piezoOscillate(num,time,speed)

# Motors for XYZ movement            
    def move_up(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())
        self.motors.move_dx(delta_xy/1000)
        self.get_current_xy_position()
    def move_down(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())     
        self.motors.move_dx(-delta_xy/1000)
        self.get_current_xy_position()
    def move_right(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())     
        self.motors.move_dy(delta_xy/1000)
        self.get_current_xy_position()

    def move_left(self):
        delta_xy=float(self.lineEdit_xy_motor_step.text())     
        self.motors.move_dy(-delta_xy/1000)
        self.get_current_xy_position()

    def move_center(self):
        print ('goto_x_center: in function')
        self.motors.move_ax(self.center_x_FOV)
        print ('goto_x_center: done')
        print ('goto_y_center: in function')
        self.motors.move_ay(self.center_y_FOV)
        print ('goto_y_center: done')
        self.get_current_xy_position()

    def move_z_up(self):       
        delta_z=float(self.lineEdit_z_motor_step.text())
        self.motors.move_dz(delta_z/1000)
        print ('move done')
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
        print ('going home')
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
        
        print ('self.percentactual', self.percent)

        try:
            self.posrotationaryexpected = float((45./math.pi)*(math.asin(2.*(self.percent)/100.-1.)+math.pi/2.+23.*math.pi/45.))
        except:
            print ('going to 99%')
            self.exception = True
            
        if self.posrotationaryexpected  < float((45./math.pi)*(math.asin(2.*(self.pourcentmax)/100.-1.)+math.pi/2.+23.*math.pi/45.)):
            self.power_motor.goTo_abs(1,self.posrotationaryexpected) #for step of 50 microns
            #time.sleep(1)
                
        elif self.exception:
            deg=float((45./math.pi)*(math.asin(2.*99./100.-1.)+math.pi/2.+23.*math.pi/45.))
            self.power_motor.goTo_abs(1, deg)
            print ('laser maxed out at 99')
            self.exception = False
            
        
    def laserOn(self):
        self.laser.OpenLaser()
    
    def laserOff(self):
        self.laser.CloseLaser()
    
    def change_wavelength(self):
        wavelength = str(self.lineEdit_laser_output.text())
        self.laser.ChangeWavelength(wavelength)
        print ('Wavelength changed to : ', wavelength ,'nm' )
        
                
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
        self.label_pathToData.setText(str(self.save_filename))
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
        
    def write_motor_position(self):
        print('writing previous positions:...')
        position_x=self.motors.get_pos(1)
        position_y=self.motors.get_pos(2)
        position_z=self.motors.get_pos(3)
        
        strCoord=position_x+position_y+position_z
        txtCoord = open(r"C:\git-projects\multiphoton\motor_position.txt","w") 
        txtCoord.write(strCoord)
        txtCoord.close()
        print('...done!')
       
    def read_motor_position(self):
        print('reading previous positions:...')
        txtCoord = open(r"C:\git-projects\multiphoton\motor_position.txt","r") 
        tmp=txtCoord.read()
        strCoord = tmp.split('\r\n')
        position_x=strCoord[0]
        position_y=strCoord[1]
        position_z=strCoord[2]

        self.motors.move_ax(position_x)
        self.motors.move_ay(position_y)
        self.motors.move_az(position_z)
        txtCoord.close()
        print('...done!')
        