# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 07:58:44 2015

@author: flesage
"""
import pyqtgraph as pg
import Queue
import numpy as np
import scipy.interpolate
from sklearn.linear_model.tests import test_least_angle
import math
import random
from vasc_seg.ScanLines import ScanLines
import scipy.io as sio


class ChannelViewer(Queue.Queue):

    def __init__(self,name):
        Queue.Queue.__init__(self,2)
        self.imv = pg.ImageView(None,name)
        self.imv.setWindowTitle(name)
        #self.region = pg.LineSegmentROI([128, 128], [328, -328], pen=(1,9))
        
        self.lines = [];
        
        x1=random.uniform(1, 512)
        y1=random.uniform(1, 512)
        angleLine=random.uniform(0,math.pi)
        length=60;
        x2=x1+math.cos(angleLine)*length;
        y2=y1+math.sin(angleLine)*length;
        
        
        #x1=0
        #y1=0
        #x2=200;
        #y2=100;
        
        self.region = pg.LineROI([x1, y1], [x2, y2], width=1, pen=(1,9))
        self.imv.addItem(self.region)
        
        self.linescan_not_displayed=1
        self.imv.show()
        
        #self.setTestImage()
        #self.generateAutoLines()
        #self.displayLines()
        
    def showWindow(self):
        self.imv.show()
            
    def displayLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.addItem(line)
                
    def resetLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.removeItem(line)
            
    def addLines(self):
        self.resetLines()        
        x1=random.uniform(1, 512)
        y1=random.uniform(1, 512)
        angleLine=random.uniform(0,math.pi)
        length=60;
        x2=x1+math.cos(angleLine)*length;
        y2=y1+math.sin(angleLine)*length;
        self.lines.append(pg.LineROI([x1, y1], [x2, y2], width=1, pen=(1,9)))
        self.displayLines()
        
    def removeLastLine(self):
        self.resetLines()
        if (len(self.lines)>0):
            del self.lines[-1]        
            self.displayLines()
        else:
            print('not enough lines to delete')

    def generateAutoLines(self, 
                          scales=1, 
                          diam=10.0, 
                          length=25.0, 
                          tolerance=0.1):
        
        sl=ScanLines(self.getCurrentImage())
        print('image shape: '+str(np.shape(self.getCurrentImage())))
        #binary map
        sl.CalcBinaryMap(scales=scales)
        binmap=sl.GetOutputBinaryMap()

        #graphed skeleton
        sl.CalcGraphFromSeg()
        graph=sl.GetOutputGraph()
        
        #potential linescans
        sl.CalcLines(diam=diam, length=length, tolerance=tolerance)
        lines=sl.GetOutputLines()
        
        self.resetLines()
        
        for i in lines:
            
            x1=i[0,0]
            y1=i[0,1]
            
            x2=i[1,0]
            y2=i[1,1]

            self.lines.append(pg.LineROI([x1, x2], [y1, y2], width=1, pen=(1,9)))
 
    def getCurrentImage(self):
        return np.array(self.imv.image).astype('float')
        
    def setTestImage(self, path='C:/git-projects/twophoton/vasc_seg/data/im1.mat'):
        
        raw=sio.loadmat(path)['im']   
        s=250
        raw=raw[s,:,:]
        self.imv.setImage(-raw)
    
    def update(self):
        try:
            data = self.get(False)
            self.imv.setImage(-data)
            #- pour direct
        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass
        
    def toggleLinearSelection(self):
        if self.linescan_not_displayed:
            self.imv.addItem(self.region)
            self.linescan_not_displayed=0
        else:
            self.imv.removeItem(self.region)
            self.linescan_not_displayed=1
            
    def getSelectedLinePosition(self,lineNumber):
        x_e, y_e=self.lines[lineNumber].pos()
        length,width=self.lines[lineNumber].size()
        alpha_e=self.lines[lineNumber].angle()*math.pi/180     #angle in radiants of the line
        x_0=x_e-width/2*math.sin(alpha_e)           #coordinate transform to obtain origin of line
        y_0=y_e+width/2*math.cos(alpha_e)        
        delta_x= length * math.cos(alpha_e)
        delta_y= length * math.sin(alpha_e)      
        x_1=x_0+delta_x
        y_1=y_0+delta_y
        return x_0, y_0, x_1, y_1, length    
            
    def getCurrentLinePosition(self):
        x_e, y_e=self.region.pos()                  #coordinates of the lower left corner edge
        length,width=self.region.size()             #length and width of the line
        alpha_e=self.region.angle()*math.pi/180     #angle in radiants of the line
        x_0=x_e-width/2*math.sin(alpha_e)           #coordinate transform to obtain origin of line
        y_0=y_e+width/2*math.cos(alpha_e)        
        delta_x= length * math.cos(alpha_e)
        delta_y= length * math.sin(alpha_e)      
        x_1=x_0+delta_x
        y_1=y_0+delta_y
        return x_0, y_0, x_1, y_1, length

class SpeckleViewer(Queue.Queue):
    def __init__(self,name, depth):
        Queue.Queue.__init__(self,2)
        self.imv = pg.ImageView(None,name)
        self.imv.setWindowTitle(name)
        self.imv.show()
        self.first_time = True
        self.depth = depth
        self.current_index = 0

    def update(self):
        try:
            data = self.get(False)
            if self.first_time:
                self.block=np.zeros((data.shape[0],data.shape[1],self.depth))
            self.block[:,:,self.current_index]=data
            var_image = np.std(self.block,2)
            mean_image = np.mean(self.block,2)+1
            self.imv.setImage(var_image/mean_image)
            self.current_index=(self.current_index+1)%self.depth
        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass

class OCTViewer(ChannelViewer):
    def __init__(self,name):
        ChannelViewer.__init__(self,name)
        central_freq=870
        spectro_width = 122
        start_freq = central_freq-spectro_width/2
        end_freq=central_freq+spectro_width/2
        lambda_vec = np.linspace(start_freq,end_freq,1024)
        nonlin_kvec = np.flipud(2*np.pi/lambda_vec)
        lin_kvec =np.linspace(np.min(nonlin_kvec),np.max(nonlin_kvec),1024)
        # Build interpolation matrix
        self.interp_matrix = np.zeros((1024,1024))
        for i in range(1024):
            tmp=np.zeros((1,1024))
            tmp[0,i]=1
            f=scipy.interpolate.interp1d(nonlin_kvec,tmp)
            self.interp_matrix[:,i]=f(lin_kvec)

    def update(self):

        try:

            data = self.get(False).astype('float')

            # dim 0 = x
            # dim 1 = z
            # Substract mean
            data -= np.tile(np.mean(data,0),[data.shape[0],1])

            # Do interpolation
            data=np.dot(self.interp_matrix,np.transpose(data))
            # Do fft along z, assume hermitian data in z (real in f)
            data = np.absolute(np.fft.ihfft(data,None,0))

            # Display in log scale
            data=np.log(np.transpose(data)-np.min(data)+0.1)
            #self.imv.setImage(np.fliplr(data[:,10:]))
            self.imv.setImage(np.fliplr(data),False)


        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass


class PhaseViewer(ChannelViewer):
    def __init__(self,name):
        ChannelViewer.__init__(self,name)
        central_freq=870
        spectro_width = 122
        start_freq = central_freq-spectro_width/2
        end_freq=central_freq+spectro_width/2
        lambda_vec = np.linspace(start_freq,end_freq,1024)
        nonlin_kvec = np.flipud(2*np.pi/lambda_vec)
        lin_kvec =np.linspace(np.min(nonlin_kvec),np.max(nonlin_kvec),1024)
        # Build interpolation matrix
        self.interp_matrix = np.zeros((1024,1024))
        for i in range(1024):
            tmp=np.zeros((1,1024))
            tmp[0,i]=1
            f=scipy.interpolate.interp1d(nonlin_kvec,tmp)
            self.interp_matrix[:,i]=f(lin_kvec)

    def update(self):

        try:

            data = self.get(False).astype('float')

            # dim 0 = x
            # dim 1 = z
            # Substract mean
            data -= np.tile(np.mean(data,0),[data.shape[0],1])

            # Do interpolation
            data=np.dot(self.interp_matrix,np.transpose(data))
            # Do fft along z, assume hermitian data in z (real in f)
            data = np.fft.ihfft(data,None,0)
            data=data[1:,:]-data[0:-1,:]
            data = np.transpose(np.angle(data))

            # Display phase
            self.imv.setImage(np.fliplr(data))


        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass