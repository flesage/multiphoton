# -*- coding: utf-8 -*-
"""
Created on Fri Jun 26 10:29:55 2015

@author: flesage

This module contains all classes associated with the Daqmx interface
"""
import numpy as np
import posixpath
import Queue
from PyDAQmx import Task
from PyDAQmx.DAQmxTypes import *
from PyDAQmx.DAQmxConstants import *

"""
Continuous analog input task. Instantiate with number of channels, then config
according to buffer size for one image and data acquisition frequency. Data will
be generated in blocks of buffer size.
"""
class AnalogInputTask(Task):

    def __init__(self,device, ai,n_channels):
        Task.__init__(self)
        self.started = False
        self.consumers=[]
        self.decoder = None
        self.device = device
        self.n_channels=n_channels
        self.finite = False
        # First create ai task for all channels
        self.CreateAIVoltageChan(posixpath.join(device,ai),"AIs",DAQmx_Val_RSE,-5.0,5.0,DAQmx_Val_Volts,None)

    def config(self,n_pts,daq_freq,finite=False):
            self.stopTask()
            self.n_pts = n_pts
            if self.finite:
                self.CfgSampClkTiming(posixpath.join(self.device,'ao/SampleClock'),daq_freq,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,self.n_pts)
            else:
                self.CfgSampClkTiming(posixpath.join(self.device,'ao/SampleClock'),daq_freq,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.n_pts)
                buffer_size=self.n_channels*self.n_pts*2
                self.CfgInputBuffer(buffer_size)
            self.AutoRegisterEveryNSamplesEvent(DAQmx_Val_Acquired_Into_Buffer,self.n_pts,0)
            self.data = np.zeros((self.n_pts*self.n_channels,), dtype=np.int16)

    def setFinite(self,finite):
        self.finite = finite

    def setDataConsumer(self,consumer, wait, channel):

        # Consumers list contains triplet of consumer, push method and channel to push
        self.consumers.append(consumer)
        self.consumers.append(wait)
        self.consumers.append(channel)
        
    def removeDataConsumer(self,consumer):
        for i in range(len(self.consumers)):
            if self.consumers[i] == consumer:
                del self.consumers[i]
                del self.consumers[i]
                del self.consumers[i]
                break


    def setDecoder(self,decoder):

        self.decoder = decoder

    def startTask(self):

        if not self.started:
            self.StartTask()
            self.started = True

    def stopTask(self):

        if self.started:
            self.StopTask()
            self.started = False

    def clearTask(self):

        if self.started:
            self.stopTask()
        self.ClearTask()

    def EveryNCallback(self):

        # Push data to consumers requesting data with appropriate flags
        read = int32()
        self.ReadBinaryI16(self.n_pts,1,DAQmx_Val_GroupByChannel,self.data,self.n_pts*self.n_channels,byref(read),None)
        for ic in range(0,len(self.consumers),3):
            try:
                local_data =  self.data[self.n_pts*self.consumers[ic+2]:self.n_pts*(self.consumers[ic+2]+1)]
                if self.decoder is not None:
                    local_data = self.decoder.decode(local_data)
                self.consumers[ic].put(local_data,self.consumers[ic+1])
            except Queue.Full:
                pass
        return 0 # The function should return an integer


        
class OnDemandVoltageOutTask(Task):
    def __init__(self, device, ao, name):
        Task.__init__(self)

        self.device = device
        # First create ao task to send discrete on demand values
        self.CreateAOVoltageChan(posixpath.join(device,ao),name,-10.0,10.0,DAQmx_Val_Volts,None)
        self.StartTask()


    def write(self,value):
        self.WriteAnalogScalarF64(1,10.0,value,None)


class OnDemandDigitalOutTask(Task):
    def __init__(self,device, do, name):
        Task.__init__(self)
        self.device = device
        # First create ao task to send discrete on demand values
        self.CreateDOChan(posixpath.join(device,do),name,DAQmx_Val_ChanPerLine)
        self.StartTask()

    def on(self):

        data=np.ones((8,1),dtype='uint8')
        self.WriteDigitalLines(1,1,10.0,DAQmx_Val_GroupByChannel,data,None,None)
    def off(self):
        data=np.zeros((8,1),dtype='uint8')
        self.WriteDigitalLines(1,1,10.0,DAQmx_Val_GroupByChannel,data,None,None)

class PiezoTask():
    def __init__(self,device, ai,n_channels):

        self.ao_task = Task()
        self.camera_trig_task = Task()
        self.image_trig_task = Task()
        self.started = False
        self.samples = 512
        self.x = np.linspace(-np.pi, np.pi, self.samples)
        self.normSine = np.sin(self.x, dtype = np.float64) + 1
        self.frequency_hz = 10.0

    def setAmplitude(self, amplitude_volt):
        self.sinedata = self.normSine*amplitude_volt

    def setFrequency(self, frequency_hz):
        self.frequency_hz = frequency_hz

    def config(self):

        # First set sine wave with 512 points per cycle
        self.ao_task.CreateAOVoltageChan("Dev1/ao1","PiezoSineWave",-10.0,10.0, DAQmx_Val_Volts,None)
        daq_freq = float(self.frequency_hz * self.samples)
        self.ao_task.CfgSampClkTiming("", daq_freq, DAQmx_Val_Rising, DAQmx_Val_ContSamps, self.samples)

        # Then create a pulse on the same clock
        camera_freq = self.frequency_hz*4;
        self.camera_trig_task.CreateCOPulseChanFreq("Dev1/ctr0",'CameraTrig',DAQmx_Val_Hz,DAQmx_Val_Low,0,camera_freq,0.5)
        self.camera_trig_task.CfgImplicitTiming(DAQmx_Val_ContSamps,self.samples);

        # Then create a second pulse to indicate 'image' frame rate (every 4)
        self.image_trig_task.CreateCOPulseChanFreq("Dev1/ctr1",'ImageTrig',DAQmx_Val_Hz,DAQmx_Val_Low,0,self.frequency_hz,0.5)
        self.image_trig_task.CfgImplicitTiming(DAQmx_Val_ContSamps,self.samples);

        # Set triggers for synchronization
        self.camera_trig_task.CfgDigEdgeStartTrig ("/Dev1/ao/StartTrigger", DAQmx_Val_Rising);
        self.image_trig_task.CfgDigEdgeStartTrig ("/Dev1/ao/StartTrigger", DAQmx_Val_Rising);

        # Write data
        self.ao_task.WriteAnalogF64(self.samples,False,-1,DAQmx_Val_GroupByChannel, self.sinedata,None, None)

    def startTask(self):

        if not self.started:
            self.image_trig_task.StartTask()
            self.camera_trig_task.StartTask()
            self.ao_task.StartTask()
            self.started = True

    def stopTask(self):

        if self.started:
            self.image_trig_task.StopTask()
            self.camera_trig_task.StopTask()
            self.ao_task.StopTask()
            self.started = False

    def clearTask(self):

        if self.started:
            self.stopTask()

        self.image_trig_task.ClearTask()
        self.camera_trig_task.ClearTask()
        self.ao_task.ClearTask()