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

class VoltageOutTask(Task):
    def __init__(self):
        Task.__init__(self)

class PO2Acq(object):
    '''
    classdocs
    '''
    def __init__(self,device, ao_eom, gate_on, gate_off, voltage_on, n_average):
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
        
    def config(self):
        # First create both channels
        freq = 1e6
        # Number of points
        self.n_pts=int((self.gate_on+self.gate_off)*freq/1e6)
        # Ramp to eom
        self.eom_data=np.zeros((self.n_pts,self.n_average))
        self.n_pts_on=int(self.gate_on*freq/1e6)
        self.eom_data[0:self.n_pts_on,:]=self.voltage_on
        self.eom_data=self.eom_data.flatten()
        
        
        self.po2_task = VoltageOutTask()
        self.po2_task.CreateAOVoltageChan(posixpath.join(self.device,self.ao_po2),"PO2",-10.0,10.0,DAQmx_Val_Volts,None)
        self.po2_task.CfgSampClkTiming("",freq,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,int(self.n_pts*self.n_average))
        if self.ai_task is not None:
            self.ai_task.config(int(self.n_pts*self.n_average),freq,finite=True)
            self.ai_task.CfgDigEdgeStartTrig (posixpath.join(self.device,"ao/StartTrigger"), DAQmx_Val_Falling)

    def setSynchronizedAITask(self, ai_task)
        self.ai_task = ai_task
        
    def start(self):
        if self.ai_task is not None:
            self.ai_task.start()

        read = int32()
        # Task is started by Write which will trigger Ai task
        self.po2_task.WriteAnalogF64(self.n_pts*self.n_average,True,-1,DAQmx_Val_GroupByChannel,
            self.eom_data,byref(read),None)
        
        # wait until write is completeled
        isDone = False
        isDoneP = c_ulong()
        while not isDone:
            err = self.po2_task.IsTaskDone(byref(isDoneP))
            isDone = isDoneP.value != 0
            
        self.po2_task.StopTask()
        
        # make sure read is also completed
        isDone = False
        isDoneP = c_ulong()
        while not isDone:
            err = self.ai_task.IsTaskDone(byref(isDoneP))
            isDone = isDoneP.value != 0
            
        self.ai_task.stopTask()
        
    def close(self):
        if self.ai_task is not None:
            self.ai_task.clearTask()

        self.po2_task.StopTask()
        self.po2_task.ClearTask()


        