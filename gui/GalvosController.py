# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 10:48:31 2015

@author: flesage
"""

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


from base import liomio
import numpy as np
import icons_rc
from base.liomio import DataSaver
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import posixpath
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
        uic.loadUi(os.path.join(basepath,"galvos_form.ui"), self)

        # Mechanical shutters
        self.shutter2ph_closed = True                      
        self.shutter3ph_closed = True                      
        self.pushButton_shutter_2ph.clicked.connect(self.toggle_shutter2ph)
        self.pushButton_shutter_3ph.clicked.connect(self.toggle_shutter3ph)
        self.pushButton_shutter_2ph.setStyleSheet("background-color: pale gray");
        self.pushButton_shutter_3ph.setStyleSheet("background-color: pale gray");

        
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
        
        #Saving:
        self.pushButton_select_save_directory.clicked.connect(self.select_saving_directory)
        self.lineEdit_mouse_name.setEnabled(False)

        self.checkBox_enable_save.setEnabled(False)
        
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
        self.pushButton_show_line.clicked.connect(self.toggle_line)
        self.toggleLineFlag=0;
        self.pushButton_get_line_position.clicked.connect(self.update_linescan)
        self.lineEdit_nt.textChanged.connect(self.update_linescan)
        self.lineEdit_linerate_LS.textChanged.connect(self.update_linescan)
        self.pushButton_snap_angio.setEnabled(False)        
        self.pushButton_start_linescan.setEnabled(False)
        self.pushButton_stop_linescan.setEnabled(False)
        self.pushButton_get_line_position.setEnabled(False)
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

        #Save folder:
        self.pushButton_set_save_name.setEnabled(False)
        self.lineEdit_save_name.setEnabled(False)
        self.pushButton_set_folder.clicked.connect(self.set_save_folder)
        self.pushButton_set_save_name.clicked.connect(self.create_new_folder)
        
        # 3P Wheel:
        self.flagWheel = 1
        
        #Multiple lines:
        self.pushButtonAddLine.clicked.connect(self.addLines)
        self.pushButtonDeleteLastLine.clicked.connect(self.removeLastLine)
        self.pushButtonShowLines.clicked.connect(self.displayLines)
        self.toggleDisplayLines = 1
        self.pushButtonStartMultipleLineAcq.clicked.connect(self.start_multiple_lines)
        self.pushButtonStopMultipleLineAcq.clicked.connect(self.stop_multiple_lines)
        self.pushButtonStopMultipleLineAcq.setEnabled(False)
        self.pushButtonStartMultipleLineAcq.setEnabled(False)
        self.pushButtonShowLines.setEnabled(False)
        self.pushButtonDeleteLastLine.setEnabled(False)
        
    #IMAGE VIEWER FUNCTIONS:    
        
    def setImageViewer(self,viewer):
        self.viewer = viewer
    
    def setImageViewer2(self,viewer2):
        self.viewer2 = viewer2
 
    def showChannel1(self):
        self.viewer.showWindow()    

    def showChannel2(self):
        self.viewer2.showWindow() 
 
    #MULTIPLE LINE FUNCTIONS:
    
    def addLines(self):
        self.viewer.addLines()
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
            self.pushButtonDeleteLastLine.setEnabled(True)
        else:
            self.pushButtonStartMultipleLineAcq.setEnabled(False)
            self.pushButtonShowLines.setEnabled(False)
            self.pushButtonDeleteLastLine.setEnabled(False)

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
        self.saveName=str(self.save_directory)+str(self.lineEdit_save_name.text())+'.hdf5'
        f = h5py.File(self.saveName, "w")
        
    #POWER CURVE:    
        
    def togglePowerCurve(self,status):
        self.checkBoxPowerCurve.setEnabled(status)

    def clearCurve(self):
        self.positionVector=[]
        self.powerVector=[]
        self.togglePowerCurve(False)
        
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

    def generatePowerCurve(self):
        self.positionVector = np.array(self.positionVector)
        self.powerVector = np.array(self.powerVector).astype(float)
        self.zstep=float(self.lineEdit_zstep.text())
        self.depthVector=np.arange(0,1000.0,self.zstep)/1000.0
        
        f2 = interp1d(self.positionVector, self.powerVector, kind='cubic',fill_value='extrapolate')
        self.powerCurve=f2(self.depthVector)
        self.powerCurve[self.powerCurve >= 100] = 100
        
    #LINE SCAN FUNCTIONS:

    def toggle_line(self):
        self.viewer.toggleLinearSelection()
        if self.toggleLineFlag==0:
            self.pushButton_show_line.setText('Hide Line')
            self.toggleLineFlag=1
            self.toggle_linescanDisplayButton(False)
        else:        
            self.pushButton_show_line.setText('Show Line')
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
        linescan_width=self.get_line_pos()
        self.lineEdit_line_scan_width.setText(str(linescan_width))
                
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
        
    def get_line_pos(self):
        linescan_px_x_1, linescan_px_y_1, linescan_px_x_2, linescan_px_y_2, linescan_length_px = self.viewer.getCurrentLinePosition()

        nx=int(self.lineEdit_nx.text())   
        ny=int(self.lineEdit_ny.text())   
        width=float(self.lineEdit_width.text())          
        height=float(self.lineEdit_height.text())      
        
        linescan_x_1_um=linescan_px_x_1/nx*width
        linescan_x_2_um=linescan_px_x_2/nx*width
        linescan_y_1_um=linescan_px_y_1/ny*height
        linescan_y_2_um=linescan_px_y_2/ny*height
        
        linescan_width = math.sqrt(math.pow((linescan_x_2_um-linescan_x_1_um),2)+math.pow((linescan_y_2_um-linescan_y_1_um),2))
        return linescan_width
   
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
        self.label_power2P.setText(str(value)+' %')
        eom_voltage=2.0*value/100.0
        print eom_voltage
        self.power_ao_eom.write(eom_voltage)
    
    def setPower3ph(self,value):
        # Input value is 0-100%, needs to be mapped to 0-2V for EOM input
        if self.flagWheel == 1:
            self.label_power3P.setText(str(value)+' %')
            wheel3P_angle=360.0*value/100.0
            command = self.wheel3P_angle_offset+wheel3P_angle
            self.flagWheel = 0
            thread = threading.Thread(target=self.wheel_run, args=(command,))
            thread.start()
        
    def setMinimumPower3ph(self):
        print('set minimum...')
        self.wheel3P_angle_offset=self.wheel3P_angle_offset+(360.0*self.horizontalScrollBar_power3ph.value()/100.0-10)
        print('half done...')
        self.horizontalScrollBar_power3ph.setValue(0)
        print('done!')
        
    def wheel_run(self,command):
        print "Starting rotation:"
        self.thorlabs.setpos(command)
        self.flagWheel = 1
        
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
        
    def get_current_z_depth(self):
        print 'in current z depth'
        print self.brainPosSetFlag
        if self.brainPosSetFlag==1:
            print 'brain set...'
            self.abs_pos=self.motors.get_pos(3)
            self.currentZPos=float(self.brain_pos)-float(self.abs_pos)
            print self.currentZPos            
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
        
    def startlinescan(self):
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
        self.pushButton_show_line.setEnabled(False)
        self.pushButton_get_line_position.setEnabled(False)
        
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
        
        npts=int(self.lineEdit_nr.text())   
        n_lines=int(self.lineEdit_nt.text())   
        n_extra=int(self.lineEdit_extrapoints_LS.text())   
        line_rate=int(self.lineEdit_linerate_LS.text())   
        
        linescan_width_um = math.sqrt(math.pow((x_e_um-x_0_um),2)+math.pow((y_e_um-y_0_um),2))
        
        shift_display=self.linescan_shift_display
                
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
            self.ai_task.setDataConsumer(self.data_saver,True,0)
            self.ai_task.setDataConsumer(self.data_saver,True,1)
            self.data_saver.startSaving()
        self.galvos.startTask()

    def stoplinescan(self):
        self.toggle_shutter2ph()
        self.viewer.toggleLinearSelection()
        
        self.pushButton_start_linescan.setEnabled(True)
        self.pushButton_stop_linescan.setEnabled(False)
        self.pushButton_show_line.setEnabled(True)
        self.pushButton_get_line_position.setEnabled(True)
        if self.checkBox_enable_save.isChecked():
            self.data_saver.stopSaving()
        self.galvos.stopTask()    
        self.galvos_stopped = True
                        
        self.galvos.stopTask()    
        self.galvos_stopped = True
        self.pushButton_start.setEnabled(True)
        
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
            self.ai_task.setDataConsumer(self.data_saver,True,0)
            self.ai_task.setDataConsumer(self.data_saver,True,1)
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
            self.ai_task.setDataConsumer(self.data_saver,True,0)
            self.ai_task.setDataConsumer(self.data_saver,True,1)
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
                #self.data_saver.setBlockSize(512)
            print('stop saving done!')
            
            
    def make_connection_snapshot(self, slider_object):
        slider_object.connect(self.stop_snapshot)
    
    #MULTIPLE LINE SCANS
    
    def start_multiple_lines(self):
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
            self.ai_task.setDataConsumer(self.data_saver,True,0)
            self.ai_task.setDataConsumer(self.data_saver,True,1)
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

    def make_connection_multiple_lines(self,slider_object):
        print('making connection multiple lines...')
        slider_object.connect(self.nextLine)
        print('...done!')

    @pyqtSlot()       
    def nextLine(self):
        print('in NEXTLINE')
        if self.checkBox_enable_save.isChecked():
            self.data_saver.stopSaving()
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
                    self.ai_task.setDataConsumer(self.data_saver,True,0)
                    self.ai_task.setDataConsumer(self.data_saver,True,1)
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

    #STACK
    
    def start_stack_thread(self):
        if self.checkBoxPowerCurve.isChecked():
            self.generatePowerCurve()
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
            self.ai_task.setDataConsumer(self.data_saver,True,0)
            self.ai_task.setDataConsumer(self.data_saver,True,1)
            print 'starting saving'
            self.get_current_z_depth()
            self.lineEdit_current_depth.setText(str(self.currentZPos))
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

        if self.checkBoxStack3P.isChecked():
            currentPowerValue=self.powerCurve[0]
            self.setPower3ph(currentPowerValue)
            self.horizontalScrollBar_power3ph.setValue(currentPowerValue)
            txt= str(round(currentPowerValue,1)) + '%'
            self.lineEdit_current_power.setText(txt)
        
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
                    print('power value: ' + str(round(currentPowerValue,1)) + '%')
                    if self.checkBoxStack2P.isChecked():
                        self.setPower2ph(currentPowerValue)
                        self.horizontalScrollBar_power2ph.setValue(currentPowerValue)
                        
                    if self.checkBoxStack3P.isChecked():
                        self.setPower3ph(currentPowerValue)
                        self.horizontalScrollBar_power3ph.setValue(currentPowerValue)
                    txt= str(round(currentPowerValue,1)) + '%'
                    self.lineEdit_current_power.setText(txt)
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
            if self.checkBoxStack3P.isChecked():
                self.setPower3ph(0)
                self.horizontalScrollBar_power3ph.setValue(0)
            if self.checkBox_enable_save.isChecked():
                print('stop saving...')
                self.data_saver.stopSaving()
                #self.data_saver.setBlockSize(512)
                print('stop saving done!')
        #time.sleep(2)
        
    def make_connection_offset_X(self):
        print('in make connection offset')
        self.horizontalScrollBar_line_scan_shift_x.valueChanged.connect(self.galvos.move_offset_X)        
        print('done: in make connection offset')

    def make_connection_offset_Y(self):
        print('in make connection offset')
        self.horizontalScrollBar_line_scan_shift_y.valueChanged.connect(self.galvos.move_offset_Y)        
        print('done: in make connection offset')
        
    def make_connection_offset_Display(self):
        print('in make connection offset')
        self.horizontalScrollBar_line_scan_shift_display.valueChanged.connect(self.galvos.move_offset_display)        
        print('done: in make connection offset')
    
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
        