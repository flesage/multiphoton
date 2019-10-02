'''
Created on 16 mai 2019

@author: LiomW17
'''
import posixpath
import PyDAQmx
from PyDAQmx import Task
from PyDAQmx.DAQmxTypes import *
from PyDAQmx.DAQmxConstants import *
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5 import QtCore

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
        print('signal helper created!')
        self.signal_helper = SignalHelper()            
        
    def DoneCallback(self,status):
        self.StopTask()
        self.signal_helper.on_ao_done()

        return 0 # The function should return an integer 

class PO2Acq(object):
    '''
    classdocs
    '''
    def __init__(self,device, ao_eom, gate_on, gate_off, voltage_on, n_average,freq):
        self.device = device
        self.ao_po2 = ao_eom
        self.po2_task = None     
        self.ai_task = None  
        self.gate_on = gate_on
        self.gate_off = gate_off
        self.voltage_on = voltage_on 
        if (self.voltage_on > 2):
            self.voltage_on = 2
        self.n_average = n_average
        self.freq = freq
        
        #2e5
        
    def config(self):
        # First create both channels
        freq=self.freq
        # Number of points
        self.n_pts=int((self.gate_on+self.gate_off)*freq/1e6)
        # Ramp to eom
        self.eom_data=np.zeros((self.n_pts,self.n_average))
        self.n_pts_on=int(self.gate_on*freq/1e6)
        print('time ON:'+str(self.n_pts_on))
        self.eom_data[0:self.n_pts_on,:]=self.voltage_on
        self.eom_data=self.eom_data.flatten('F')
        
        
        self.po2_task = VoltageOutTask()
        self.po2_task.createSignalHelper()
        self.po2_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_po2),"PO2",-10.0,10.0,DAQmx_Val_Volts,None)
        self.po2_task.CfgSampClkTiming("",freq,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,int(self.n_pts*self.n_average))
        self.po2_task.AutoRegisterDoneEvent(0)

        if self.ai_task is not None:
            print('config start trigger')
            self.ai_task.config(int(self.n_pts*self.n_average),freq,finite=True)
            self.ai_task.CfgDigEdgeStartTrig (posixpath.join(self.device,"ao/StartTrigger"), DAQmx_Val_Falling)

    def setSynchronizedAITask(self, ai_task):
        self.ai_task = ai_task
        
    def writeOnce(self):
        read = int32()
        self.po2_task.WriteAnalogF64(self.n_pts*self.n_average,False,-1,DAQmx_Val_GroupByChannel,
            self.eom_data,byref(read),None)        
        
    def start(self):
        print('starting eom...')
        if self.ai_task is not None:
            print('starting ai')
            self.ai_task.startTask()
        # Task is started by Write which will trigger Ai task
        self.po2_task.StartTask()
        # wait until write is completeled
        #isDone = False
        #isDoneP = c_ulong()
        #while not isDone:
        #    self.po2_task.IsTaskDone(byref(isDoneP))
        #    isDone = isDoneP.value != 0
        
        #print('stopping eom...')
        #self.po2_task.StopTask()
        #print('eom stopped!')

        #print('stopping ai')
        #self.ai_task.stopTask()
        #print('ai stopped')

    def close(self):
        #if self.ai_task is not None:
        #    self.ai_task.clearTask()

        self.po2_task.StopTask()
        self.po2_task.ClearTask()


class PO2GatedAcq(object):
    '''
    classdocs
    '''
    def __init__(self,device, ao_eom):
        self.device = device
        self.ao_po2 = ao_eom
        self.po2_task = None     
        self.ai_task = None  
        self.read = int32()
        
    def config(self,galvo_freq, amplitude,offset, n_average,freq_daq):
        if self.po2_task is not None:
            self.po2_task.StopTask()
            self.po2_task.ClearTask()  
        self.galvo_freq = galvo_freq
        self.n_average = n_average
        self.sine_voltage = amplitude
        self.offset=offset
        # First create sine wave to scan through the slit, assume center (zero volt) is slit
        # Number of points
        self.n_pts=int((1/(2*galvo_freq))*freq_daq)
        # Sine ramp, each cycle goes twice through the slit, thus 2 averages per cycle
        # We then tile to get all averages
        data=np.sin(np.linspace(-np.pi/2,3*np.pi/2,2*self.n_pts))
        
        #data=np.sin(np.linspace(0,4*np.pi/2,2*self.n_pts))

        self.gated_data=np.tile(data,self.n_average/2)
        self.gated_data=self.gated_data.flatten()*self.sine_voltage+self.offset
        #val=np.sin(3*np.pi/2)*self.sine_voltage+self.offset
        #self.vector_data=np.ones(100)*val
        #self.vector_data_zeros=np.zeros(100)
        #self.gated_data=np.append(self.gated_data,self.vector_data)
        #self.gated_data=np.append(self.gated_data,self.vector_data_zeros)
        
        self.po2_task = VoltageOutTask()
        self.po2_task.createSignalHelper()
        self.po2_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_po2),"3P-PO2",-10.0,10.0,DAQmx_Val_Volts,None)
        self.po2_task.CfgSampClkTiming("",freq_daq,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,int(self.n_pts*self.n_average))
        self.po2_task.AutoRegisterDoneEvent(0)

        if self.ai_task is not None:
            print('config start trigger')
            self.ai_task.config(int(self.n_pts*self.n_average),freq_daq,finite=True)
            self.ai_task.CfgDigEdgeStartTrig (posixpath.join(self.device,"ao/StartTrigger"), DAQmx_Val_Falling)

    def configOnDemand(self):  
        self.po2_task = VoltageOutTask()
        self.po2_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_po2),"3P-PO2",-10.0,10.0,DAQmx_Val_Volts,None)

    def moveOnDemand(self,x):
        data=np.zeros((1,1))
        data[0]=x/1000
        self.po2_task.WriteAnalogF64(1,True,-1,DAQmx_Val_GroupByChannel,
                                            data,byref(self.read),None)
        
    def setSynchronizedAITask(self, ai_task):
        self.ai_task = ai_task
        
    def writeOnce(self):
        read = int32()
        self.po2_task.WriteAnalogF64(self.n_pts*self.n_average,False,-1,DAQmx_Val_GroupByChannel,
            self.gated_data,byref(read),None)        
        
        
    def start(self):
        print('starting eom...')
        if self.ai_task is not None:
            print('starting ai')
            self.ai_task.startTask()
            print('ai started!')
        # Task is started by Write which will trigger Ai task
        self.po2_task.StartTask()
        
        # wait until write is completeled
        #isDone = False
        #isDoneP = c_ulong()
        #while not isDone:
        #    self.po2_task.IsTaskDone(byref(isDoneP))
        #    isDone = isDoneP.value != 0
        
        #print('stopping eom...')
        #self.po2_task.StopTask()
        #print('eom stopped!')

        #print('stopping ai')
        #self.ai_task.stopTask()
        #print('ai stopped')

    def close(self):
        #if self.ai_task is not None:
        #    self.ai_task.clearTask()
        if self.po2_task is not None:
            self.po2_task.StopTask()
            self.po2_task.ClearTask()        