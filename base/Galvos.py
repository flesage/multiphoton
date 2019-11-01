# -*- coding: utf-8 -*-
"""
Created on Thu Aug 27 09:46:20 2015

@author: flesage
"""
import posixpath
import threading
import PyDAQmx
from PyDAQmx import Task
from PyDAQmx.DAQmxTypes import *
from PyDAQmx.DAQmxConstants import *
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5 import QtCore

import time

#from base.liomacq import VoltageOutTask

"""This class will set the scale for a given telescope and objective lens, to be
   calibrated for each system. Default is 100um per volt of galvo deflection
"""
class Converter():
    def __init__(self):
        self.x_um_per_volt = 100
        self.y_um_per_volt = 100

    def setScale(self,x_um_per_volt,y_um_per_volt):
        self.x_um_per_volt = x_um_per_volt
        self.y_um_per_volt = y_um_per_volt

    def voltX(self,um_meas_x):
        return um_meas_x/self.x_um_per_volt

    def voltY(self,um_meas_y):
        return um_meas_y/self.y_um_per_volt

"""This class will control the galvo mirrors movement, it is basically two AOs with helper
   functions to generate good ramps and manage triggering.
"""
class SignalHelper(QObject):
    aoDoneSignal = pyqtSignal()
 
    def __init__(self):
        QObject.__init__(self)
        
    def on_ao_done(self):
        self.aoDoneSignal.emit()    

class VoltageOutTask(Task):
    def __init__(self):
        Task.__init__(self)
        
    def createSignalHelper(self):
        self.signal_helper = SignalHelper()            
        
    def DoneCallback(self,status):
        self.signal_helper.on_ao_done()
        return 0 # The function should return an integer    

class Galvos():
    def __init__(self,device, ao_x, ao_y):
        self.device = device
        self.ao_x = ao_x
        self.ao_y = ao_y
        self.aomFlag = False    

        self.ao_task = None
        self.ai_task = None
        self.clock_task = None
        self.ext_clock = False
        self.started = False
        self.converter = Converter()
        self.read = int32()

        self.ramp_type=0
        self.nx = 256
        self.ny = 256
        self.n_extra = 0
        self.n_repeat = 1
        self.line_rate = 100
        self.daq_freq=self.line_rate*(self.nx+self.n_extra)
        self.n_pts_frame = (self.nx+self.n_extra)*self.ny
        self.center_x = 0.0
        self.center_y = 0.0
        self.shift_display = 0
        self.finite = False  
    
    def configOnDemand(self):  
        self.ao_task = VoltageOutTask()
        self.ao_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_x)+','+posixpath.join(self.device,self.ao_y),"Galvos",-10.0,10.0,DAQmx_Val_Volts,None)

    def moveOnDemand(self,x,y):
        data=np.zeros((2,1))
        data[0]=self.converter.voltX(x)
        data[1]=self.converter.voltY(y)
        self.ao_task.WriteAnalogF64(1,True,-1,DAQmx_Val_GroupByChannel,
                                            data,byref(self.read),None)
                
    def setEOMFlag(self,flag):
        self.aomFlag = flag

    def setEOMParameters(self,flag,ao_eom,daq_freq,powerPO2,powerLS,on_time):
        self.aomFlag = flag
        self.daq_freq = daq_freq
        self.powerPO2 = powerPO2
        self.powerLS = powerLS
        self.ao_eom = ao_eom
        self.gate_on = on_time
        
    def config(self):
        # First create both channels
        self.ao_task = VoltageOutTask()
        if self.aomFlag:
            self.ao_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_x)+','+posixpath.join(self.device,self.ao_y)+','+posixpath.join(self.device,self.ao_eom),"GalvosWithEOM",-10.0,10.0,DAQmx_Val_Volts,None)
        else:
            self.ao_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_x)+','+posixpath.join(self.device,self.ao_y),"Galvos",-10.0,10.0,DAQmx_Val_Volts,None)
        
        if not self.ext_clock:
            # AO will trigger everyone
            if self.finite:
                self.ao_task.createSignalHelper()
                self.ao_task.CfgSampClkTiming('',self.daq_freq,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,int(self.n_pts_frame))
                self.ao_task.AutoRegisterDoneEvent(0)
            else:
                self.ao_task.CfgSampClkTiming('',self.daq_freq,DAQmx_Val_Rising,DAQmx_Val_ContSamps,int(self.n_pts_frame))
                self.ao_task.CfgOutputBuffer(2*int(self.n_pts_frame))
                self.ao_task.SetWriteRegenMode(DAQmx_Val_DoNotAllowRegen)
                
            if self.ai_task is not None:
                self.ai_task.config(int(self.n_pts_frame),self.daq_freq)
                self.ai_task.CfgDigEdgeStartTrig (posixpath.join(self.device,"ao/StartTrigger"), DAQmx_Val_Falling)
        else:
            # Counter clock will trigger everyone
            self.clock_task = Task()
            self.clock_task.CreateCOPulseChanFreq(posixpath.join(self.device,self.camera_clock),"",DAQmx_Val_Hz,DAQmx_Val_Low,0.0,self.daq_freq,0.50)
            self.clock_task.CfgImplicitTiming(DAQmx_Val_ContSamps,1000)

            if self.finite:
                self.ao_task.createSignalHelper()                
                self.ao_task.CfgSampClkTiming(posixpath.join(self.device,self.camera_clock_pfi),self.daq_freq,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,int(self.n_pts_frame))
                self.ao_task.AutoRegisterDoneEvent(0)
            else:
                self.ao_task.CfgSampClkTiming(posixpath.join(self.device,self.camera_clock_pfi),self.daq_freq,DAQmx_Val_Rising,DAQmx_Val_ContSamps,int(self.n_pts_frame))
                self.ao_task.CfgOutputBuffer(2*int(self.n_pts_frame))
                self.ao_task.SetWriteRegenMode(DAQmx_Val_DoNotAllowRegen)

            if self.ai_task is not None:
                self.ai_task.config(int(self.n_pts_frame),self.daq_freq)
                self.ai_task.CfgDigEdgeStartTrig (posixpath.join(self.device,"ao/StartTrigger"), DAQmx_Val_Falling)

    def setExternalClock(self,flag,camera_clock,camera_clock_pfi):
        self.ext_clock = flag
        self.camera_clock=camera_clock
        self.camera_clock_pfi = camera_clock_pfi

    def setUnitConverter(self,converter):

        self.converter = converter

    def setSynchronizedAITask(self,a_task):

        self.ai_task = a_task
        self.ai_task.setDecoder(self)

    def setFinite(self,finite):
        self.finite = finite
        if self.ai_task is not None:
            self.ai_task.setFinite(finite)

    def setSawToothRamp(self,x0,y0,xe,ye,nx,ny,n_extra,n_repeat,line_rate):

        if self.started:
            self.stopTask()
        self.ramp_type = 0
        self.nx = nx
        self.ny = ny
        self.n_extra = n_extra
        self.n_repeat = n_repeat
        self.line_rate=line_rate

        # Fast axis is always set to be x
        ramp_x = np.concatenate((np.linspace(x0,xe,nx),self.getPolyReturn(x0,xe,nx,x0,xe,nx,n_extra)))
        ramp_y = np.linspace(y0,ye,ny)

        full_x=self.converter.voltX(np.tile(ramp_x,[1,ny*n_repeat]))
        full_y=self.converter.voltY(np.tile(ramp_y,[n_repeat*(nx+n_extra),1]).flatten('F').reshape(1,(nx+n_extra)*ny*n_repeat))
        self.ramps=np.concatenate((full_x,full_y))

        self.daq_freq=int(self.line_rate*(self.nx+self.n_extra))
        # This is to chunk the repeats together for easier analysis
        self.n_pts_frame = int((self.nx+self.n_extra)*self.ny)
        self.config()

    def setTriangularRamp(self,x0,y0,xe,ye,nx,ny,n_extra,n_repeat,line_rate):
        if self.started:
            self.stopTask()
        # Need a pair value for ny
        if ny % 2 == 1:
            ny = ny+1

        self.ramp_type = 1
        self.nx = nx
        self.ny = ny
        self.n_extra = n_extra
        self.n_repeat = n_repeat
        self.line_rate=line_rate

        # Need to build double ramp here
        flyback_1=self.getPolyReturn(x0,xe,nx,xe,x0,nx,n_extra)
        flyback_2=self.getPolyReturn(xe,x0,nx,x0,xe,nx,n_extra)
        ramp_x = np.concatenate((np.linspace(x0,xe,nx),flyback_1,np.linspace(xe,x0,nx),flyback_2))
        ramp_y = np.linspace(y0,ye,ny)

        full_x=self.converter.voltX(np.tile(ramp_x,[1,ny/2*n_repeat]))
        full_y=self.converter.voltY(np.tile(ramp_y,[n_repeat*(nx+n_extra),1]).flatten('F').reshape(1,(nx+n_extra)*ny*n_repeat))
        self.ramps=np.concatenate((full_x,full_y))

        self.daq_freq=int(self.line_rate*(self.nx+self.n_extra))
        # This is to chunk the repeats together for easier analysis
        self.n_pts_frame = int((self.nx+self.n_extra)*self.ny)
        self.config()

    def setLineRamp(self,x0,y0,xe,ye,npts,n_lines,n_extra,line_rate,shift_display):

        self.shift_display=shift_display
        if self.started:
            self.stopTask()
        # Need a pair value for ny
        if n_lines % 2 == 1:
            n_lines = n_lines-1

        self.ramp_type = 2
        self.nx = npts
        self.ny = n_lines
        self.n_extra = n_extra
        self.n_repeat = 1
        self.line_rate = line_rate
        
        
        if self.aomFlag:
            totalNumPoints=int(np.floor(self.daq_freq/self.line_rate))
            self.nx=int(np.round((1-self.ratioLS_PO2)*totalNumPoints))
            self.n_extra=totalNumPoints-self.nx

        flybackUp_x=self.getPolyReturn(x0,xe,self.nx,xe,x0,self.nx,self.n_extra)
        flybackDown_x=self.getPolyReturn(xe,x0,self.nx,x0,xe,self.nx,self.n_extra)
        flybackUp_y=self.getPolyReturn(y0,ye,self.nx,ye,y0,self.nx,self.n_extra)
        flybackDown_y=self.getPolyReturn(ye,y0,self.nx,y0,ye,self.nx,self.n_extra)
        # Need to build double ramp here
        ramp_x = np.concatenate((np.linspace(x0,xe,self.nx),flybackUp_x,np.linspace(xe,x0,self.nx),flybackDown_x))
        ramp_y = np.concatenate((np.linspace(y0,ye,self.nx),flybackUp_y, np.linspace(ye,y0,self.nx),flybackDown_y))

        if self.aomFlag:
            ramp_eom=np.zeros((totalNumPoints,self.ny))
            self.n_pts_on=int(self.gate_on*self.daq_freq/1e6)
            startLineVoltage=int((self.ratioLS_PO2/2)*totalNumPoints)
            endLineVoltage=int((1-self.ratioLS_PO2/2)*totalNumPoints)
            
            ramp_eom[startLineVoltage:endLineVoltage,:]=self.powerLS
            ramp_eom[endLineVoltage+1:endLineVoltage+self.n_pts_on+1,:]=self.powerPO2
            ramp_eom=ramp_eom.flatten('F')
            full_eom=np.tile(ramp_eom,[1,n_lines/2])
            self.ny=self.ny/4

        full_x=self.converter.voltX(np.tile(ramp_x,[1,n_lines/2]))
        full_y=self.converter.voltY(np.tile(ramp_y,[1,n_lines/2]))
        
        if self.aomFlag:
            self.ramps=np.concatenate((full_x,full_y,full_eom))
        else:
            self.ramps=np.concatenate((full_x,full_y))
            self.daq_freq=int(self.line_rate*(self.nx+self.n_extra))
            # This is to chunk the repeats together for easier analysis
            self.n_pts_frame = int((self.nx+self.n_extra)*self.ny)
            
        self.config()
        
        if self.aomFlag:
            return self.nx,self.n_extra,self.ny
        print 'line ramp set'

    def getPolyReturn(self,line1_VoltStart, line1_VoltEnd, n_pts_line1, line2_VoltStart, line2_VoltEnd, n_pts_line2, n_extra_pts):
        '''Returns polynomial for galvo ramp to interpolate between two ramps. This
        function solves the polynomial return so that: start and end values are
        the end of line 1 et start of line 2. Slope at the start = slope of line
        1. Slope at the end = slope of line 2.
        n_extra_pts is the number of points used in the polynomial interpolation. '''

        # Force interpretation as float to avoid issues below
        line1_VoltStart = float(line1_VoltStart)
        line2_VoltStart = float(line2_VoltStart)

        # Polynomial equation left-side
        aEqu = np.reshape(np.array([1,1,1,1]),[4,1])
        bEqu = np.reshape(np.array([1,n_extra_pts,pow(n_extra_pts,2),pow(n_extra_pts,3)]),[4,1])
        cEqu = np.reshape(np.array([0,1,2,3]),[4,1])
        dEqu = np.reshape(np.array([0,1,2*n_extra_pts,3*pow(n_extra_pts,2)]),[4,1])
        # Polynomial equation right-side
        polyEqu = np.array([line1_VoltEnd, line2_VoltStart, (line1_VoltEnd-line1_VoltStart)/n_pts_line1, (line2_VoltEnd-line2_VoltStart)/n_pts_line2])
        # Solve poly equation
        x=np.linalg.solve(np.transpose(np.hstack((aEqu, bEqu, cEqu, dEqu))),np.transpose(polyEqu))
        if n_extra_pts > 0:
            yRampReturn = np.linspace(2, n_extra_pts-1, n_extra_pts)
            y = x[0] + x[1]*yRampReturn + x[2]*pow(yRampReturn,2) + x[3]*pow(yRampReturn,3)
        else:
            y=np.empty()
        return y

    def continuousWrite(self):        
        # Writes continuously and will adapt according to the center position:
        while self.started:
            # Divide in n_repeat chunks
            for i in range(self.n_repeat):
                current_ramp = self.ramps[:,i*self.n_pts_frame:(i+1)*self.n_pts_frame]
                self.ao_task.WriteAnalogF64(current_ramp.shape[1],True,-1,DAQmx_Val_GroupByChannel,
                                            (current_ramp+np.tile([[self.converter.voltX(self.center_x)],[self.converter.voltY(self.center_y)]],[1,current_ramp.shape[1]])).flatten(),byref(self.read),None)
                if self.started == False:
                    break

    def writeOnce(self):
        # Writes the ramps once
        self.ao_task.WriteAnalogF64(self.ramps.shape[1],False,-1,DAQmx_Val_GroupByChannel,
            (self.ramps+np.tile([[self.converter.voltX(self.center_x)],[self.converter.voltY(self.center_y)]],[1,self.ramps.shape[1]])).flatten(),byref(self.read),None)

    @pyqtSlot(int)
    def move_offset_X(self, center_x):
        self.center_x = center_x
         
    @pyqtSlot(int)
    def move_offset_Y(self, center_y):
        self.center_y = center_y      

    def move(self,center_x,center_y):
        print('move called!')
        self.center_x = center_x
        self.center_y = center_y

    def startTask(self):

        if self.finite:
            # Need to loop over n_repeat for finite over here
            if self.ai_task is not None:
                self.ai_task.startTask()
            self.writeOnce()
            self.ao_task.StartTask()
#            self.ao_task.WaitUntilTaskDone(DAQmx_Val_WaitInfinitely)

        else:

            if not self.started:
                if self.ai_task is not None:
                    self.ai_task.startTask()
                # Thread to write data to AOs
                # When using external clock, this is naturally synchronized
                # since there is no clock output.
                # It would be cleaner to trig on counter.
                self.started = True
                #self.writeOnce()
                #self.ao_task.StartTask()

                self.t1 = threading.Thread(target=self.continuousWrite)
                self.t1.start()

        if self.clock_task is not None:
            self.clock_task.StartTask()


    def stopTask(self):
        # This will never be needed since starttask waits until done
        # Maybe work on logic here
        if self.finite:
            self.ao_task.StopTask()
            if self.ai_task is not None:
                self.ai_task.stopTask()
        else:
            if self.started:
                if self.ai_task is not None:
                    self.ai_task.stopTask()
                self.started = False
                self.t1.join()
                self.ao_task.StopTask()
                self.ao_task.ClearTask()
                #print('Tasks cleared!')

        # Not clear this needs to be done here.
        if self.clock_task is not None:
            self.clock_task.StopTask()
            self.clock_task.ClearTask()

    def clearTask(self):

        if self.started:
            self.stopTask()
        if self.ai_task is not None:
            self.ai_task.clearTask()
        self.ao_task.ClearTask()

    def decode(self,data):

        if self.ramp_type == 0:
            image=np.reshape(data,(self.ny,(self.nx+self.n_extra)))

        elif self.ramp_type == 1:
            image=np.reshape(data,(self.ny,(self.nx+self.n_extra)))

        elif self.ramp_type == 2:            
            image=np.reshape(data,(self.ny,(self.nx+self.n_extra)))

        return image

