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
from PIL import Image
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from scipy.optimize import curve_fit

#from matplotlib import pyplot as plt


class LineScanROI(pg.graphicsItems.ROI.ROI):
 
    def __init__(self, pos1, pos2, width, **args):
        pos1 = pg.Point(pos1)
        pos2 = pg.Point(pos2)
        d = pos2-pos1
        l = d.length()
        ang = pg.Point(1, 0).angle(d)
        ra = ang * np.pi / 180.
        c = pg.Point(-width/2. * math.sin(ra), -width/2. * math.cos(ra))
        pos1 = pos1 + c
        
        pg.graphicsItems.ROI.ROI.__init__(self, pos1, size=pg.Point(l, width), angle=ang, **args)
        self.addScaleRotateHandle([0, 0.5], [1, 0.5])
        self.addScaleRotateHandle([1, 0.5], [0, 0.5])
        
class LineScanROI_perp(pg.graphicsItems.ROI.ROI):
 
    def __init__(self, pos1, pos2, width, **args):
        pos1 = pg.Point(pos1)
        pos2 = pg.Point(pos2)
        d = pos2-pos1
        l = d.length()
        ang = pg.Point(1, 0).angle(d)
        ra = ang * np.pi / 180.
        c = pg.Point(-width/2. * math.sin(ra), -width/2. * math.cos(ra))
        pos1 = pos1 + c
        
        pg.graphicsItems.ROI.ROI.__init__(self, pos1, size=pg.Point(l, width), angle=ang, **args)
        self.addScaleRotateHandle([0, 0.5], [1, 0.5])
        self.addScaleRotateHandle([1, 0.5], [0, 0.5])        


class PointROI(pg.graphicsItems.ROI.EllipseROI):

    def __init__(self, pos, size, **args):
        pg.graphicsItems.ROI.ROI.__init__(self, pos, size, **args)
        self.aspectLocked = True

class PointForLineROI(pg.graphicsItems.ROI.EllipseROI):

    def __init__(self, pos, size, **args):
        pg.graphicsItems.ROI.ROI.__init__(self, pos, size, **args)
        self.aspectLocked = True

class RectROI(pg.graphicsItems.ROI.ROI):
   
    def __init__(self, pos, size, centered=False, sideScalers=False, **args):
        #QtGui.QGraphicsRectItem.__init__(self, 0, 0, size[0], size[1])
        pg.graphicsItems.ROI.ROI.__init__(self, pos, size, **args)
        if centered:
            center = [0.5, 0.5]
        else:
            center = [0, 0]
            
        #self.addTranslateHandle(center)
        self.addScaleHandle([1, 1], center)
        if sideScalers:
            self.addScaleHandle([1, 0.5], [center[0], 0.5])
            self.addScaleHandle([0.5, 1], [0.5, center[1]])




class po2Viewer(Queue.Queue):

    def __init__(self, name):
        
        Queue.Queue.__init__(self,2)
        
        
        self.PO2canvas = pg.GraphicsLayoutWidget()
        self.PO2canvas.setWindowTitle('PO2 viewer')
        self.PO2AbsCurve = self.PO2canvas.addPlot(row=0,col=0)
        self.PO2AbsCurve.setTitle('Absolute PO2 curve')
        self.PO2AbsCurve.setLabel('left', text='Abs. Intensity')
        self.PO2AbsCurve.setLabel('bottom', text='Time [usec]')
                
        self.PO2NormCurve = self.PO2canvas.addPlot(row=0,col=1)
        self.PO2NormCurve.setTitle('Normalized PO2 curve')
        self.PO2NormCurve.setLabel('left', text='Norm. Intensity')
        self.PO2NormCurve.setLabel('bottom', text='Time [usec]')        
        
        self.PO2LogNormCurve = self.PO2canvas.addPlot(row=1,col=0)
        self.PO2LogNormCurve.setTitle('Log PO2 curve')
        self.PO2LogNormCurve.setLabel('left', text='Log. Norm. Intensity')
        self.PO2LogNormCurve.setLabel('bottom', text='Time [usec]')        #self.po2plot = pg.PlotWidget(None)
        
        self.PO2FFTCurve = self.PO2canvas.addPlot(row=1,col=1)
        self.PO2FFTCurve.setTitle('FFT PO2 curve')
        self.PO2FFTCurve.setLabel('left', text='20 log FFT')
        self.PO2FFTCurve.setLabel('bottom', text='Freq [kHz]')  
        #self.po2plot.setWindowTitle(name)
        #self.po2plot.plot(title=name)
        #self.po2plot.setTitle(name)
        #self.po2plot.setLabel('left', text='Intensity')
        #self.po2plot.setLabel('bottom', text='Time [usec]')
        #self.normFlag=0
        #self.logFlag=0
        #self.po2plot.resize(650, 450)
        self.PO2canvas.move(1200, 500)
        self.previousTracesFlag=False
        self.shiftFlag=0
    
    def update(self):
        try:
            #print('getting data...')
            data = self.get(False)
            self.counter=self.counter+1
            #averageDecayCurve=np.mean(data,2)
            #x=np.linspace(0,data/(1e6),data.shape[0])
            print('data size:'+str(data.shape))
            data_to_plot=-np.reshape(data, (self.numAverages,-1))
            print('data_to_plot size:'+str(data_to_plot.shape))
            #data=-data
            data_to_plot=np.mean(data_to_plot,0)  
            self.storedData[:,0]=data_to_plot
            self.clearPlot()
                      
            #if self.normFlag == 1:
            #    data=data-np.min(data)
            #    data=data/np.max(data)
            #if self.logFlag == 1:
            #    self.po2plot.plot(self.xAxis,np.log(data+1),pen=pg.mkPen('w', width=3))
                #self.storedData[:,self.counter]=np.log(data+1)
            #else:    
            #self.po2plot.plot(self.xAxis,data+1,pen=pg.mkPen('w', width=3))
            
            data_to_log=data_to_plot+1-np.min(data_to_plot)
            
            logTemp=np.log(data_to_log)
            logTemp=logTemp-np.min(logTemp)
            self.LogData[:,0]=logTemp/np.max(logTemp)
            
            self.normData[:,0]=data_to_plot-np.min(data_to_plot)
            self.normData[:,0]=self.normData[:,0]/np.max(self.normData[:,0])  
            
            if (self.shiftFlag==1):
                self.normData[:,0]=np.roll(self.normData[:,0],self.shiftVal,axis=0)
                self.LogData[:,0]=np.roll(self.LogData[:,0],self.shiftVal,axis=0)
                self.storedData[:,0]=np.roll(self.storedData[:,0],self.shiftVal,axis=0)
            
            dataToFFT=self.normData[:,0]
            n = len(dataToFFT)
            k = np.arange(n)
            T = n/self.PO2Acqfreq
            frq = k/T # two sides frequency range
            frq = frq[range(int(n/2))]/1000 # one side frequency range
            FFTData = np.fft.fft(dataToFFT)/n # fft computing and normalization
            self.FFTData[:,0] = 20*np.log(abs(FFTData[range(int(n/2))]))

            shapeTemp=self.storedData.shape
            shape=shapeTemp[-1]
            if (self.previousTracesFlag==0):
                shape=self.counter+1
                if ((self.counter+1)>=shapeTemp[-1]):
                    shape=shapeTemp[-1]
            
            for i in range(shape-1):
                self.PO2AbsCurve.plot(self.xAxis,self.storedData[:,i+1],pen=pg.mkPen(0.3, width=1))
                self.PO2NormCurve.plot(self.xAxis,self.normData[:,i+1],pen=pg.mkPen(0.3, width=1))
                self.PO2LogNormCurve.plot(self.xAxis,self.LogData[:,i+1],pen=pg.mkPen(0.3, width=1))
                self.PO2FFTCurve.plot(frq,self.FFTData[:,i+1],pen=pg.mkPen(0.3, width=1))
                
            self.PO2AbsCurve.plot(self.xAxis,self.storedData[:,0],pen=pg.mkPen('w', width=3))      
            self.PO2NormCurve.plot(self.xAxis,self.normData[:,0],pen=pg.mkPen('w', width=3))
            self.PO2LogNormCurve.plot(self.xAxis,self.LogData[:,0],pen=pg.mkPen('w', width=3))
            self.PO2FFTCurve.plot(frq,self.FFTData[:,0],pen=pg.mkPen('w', width=3))
            

            

            valueMaxCurve = np.amax(self.storedData[:,0])
            
            sortedCurve=np.sort(self.storedData[:,0])
            
            
            background = np.mean(sortedCurve[0:10])
            signalRange=valueMaxCurve-background
            # Get the indices of maximum element in numpy array
            positionMaxCurve = np.where(self.storedData[:,0] == valueMaxCurve)
            
            print('Position of peak: ', positionMaxCurve[0])
            print('Background :', background)
            print('Max value :', valueMaxCurve)
            print('Signal range :', signalRange)


            if (self.estimatePO2==1):
                print('real time fit...')
                if (self.shiftFlag==0):
                    data=self.storedData[15:,0]
                elif(self.shiftFlag==1):
                    data=self.storedData[15-3:,0]
                    
                time=np.arange(0,data.shape[0])/self.PO2Acqfreq
                popt, pcov = curve_fit(self.decay, time, data, p0=(np.max(data), 20e-6, np.min(data)))
                self.po2_values.append(self.po2FromModel(popt[1],self.temperature))   
                print('PO2 values:'+str(self.po2_values[-1]))            
            else:
                X=np.linspace(0,len(self.LogData[:,0]),len(self.LogData[:,0]))
                tmpFWHM=self.FWHM(X,self.LogData[:,0])
                print('tmp FWHM:'+str(tmpFWHM))
                self.po2_values.append(tmpFWHM)
                print('FWHM:'+str(self.po2_values[-1]))             


            #estimate = self.decay(time, *popt)

            print('rolling...')
            self.LogData=np.roll(self.LogData,1,axis=1)
            self.normData=np.roll(self.normData,1,axis=1)
            self.storedData=np.roll(self.storedData,1,axis=1)
            self.FFTData=np.roll(self.FFTData,1,axis=1)



                #self.storedData[:,self.counter]=data+1
            #print('data plotted!')
            #- pour direct
        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass        
        #self.Scene.sigMouseClicked.connect(self.mouseMove   

    def shiftDisplay(self,flag,val):
        self.shiftFlag=flag
        self.shiftVal=val

    def initCalculation(self,freq,fitFlag,temperature):
        self.po2_values=[]
        self.PO2Acqfreq=freq
        self.estimatePO2=fitFlag
        self.temperature=temperature
  
    def getPO2Val(self):
        return self.po2_values

    def decay(self,x, a, b, c):
        return a*np.exp(-x/b)+c

    def po2FromLifetime(self,tau):
        po2=560.895978350844*np.exp(-tau/4.390936102e-6)+158.7250245*np.exp(-tau/1.811680492e-5)-18.832277451
        return po2
    
    def po2FromModel(self,tau,T):
        p=1.265776950
        a=14.8
        kq=91.01153
        beta=19.43017
        tau0=3.8e-5
        print('*** tau:'+str(tau))
        po2=(1./(kq+beta*T))*(1./(a*(tau**p))-1./tau0)
        print('*** po2:'+str(po2))
        return po2
    
    def po2FromModelComplexe(self,tau,T):
        p=1.265776950 
        a=14.8
        kq=106.1337
        beta=18.89122
        tau0=3.96441e-5
        alpha=-5.94517e-8
        print('*** tau:'+str(tau))
        po2=(1./(kq+beta*T))*(1./(a*(tau**p))-1./(tau0+alpha*T))
        print('*** po2:'+str(po2))
        return po2
    
        
    def getEstimationPO2(self):
        return self.po2_values[-1]

    def FWHM(self,X,Y):
        half_max = 0.2
        #find when function crosses line half_max (when sign of diff flips)
        #take the 'derivative' of signum(half_max - Y[])
        d = np.sign(half_max - np.array(Y[0:-1])) - np.sign(half_max - np.array(Y[1:]))
        #plot(X[0:len(d)],d) #if you are interested
        #find the left and right most indexes
        left_idx = np.where(d > 0)[0]
        right_idx = np.where(d < 0)[-1]        
        if (len(left_idx)==0 or len(right_idx)==0):
            fwhm=np.zeros(1)
        else:
            fwhm=X[right_idx] - X[left_idx] #return the difference (full width)
        print('FWHM')
        print(len(fwhm))
        if len(fwhm)==0:
            fwhm=np.zeros(1,1)
        return fwhm[0]
    
    def showPreviousTraces(self,flag):
        print('Show previous traces: '+str(flag))
        self.previousTracesFlag=flag
        
    def getAcquisitionParameters(self,numAverages,gate_on,gate_off,freqPO2):
        self.numAverages=numAverages
        self.gate_on=gate_on #in usec
        self.gate_off=gate_off #in usec
        self.numPoints=(self.gate_on+self.gate_off)*1e-6*freqPO2
        self.timeResolution=1/freqPO2
        self.xAxis=np.arange(0,self.numPoints*self.timeResolution,self.timeResolution)
        
    def getAcquisitionParameters3P(self,numAverages,freq_galvo,freqPO2):
        self.numAverages=numAverages
        self.numPoints=(1/freq_galvo)*freqPO2/2
        self.timeResolution=1/freqPO2
        self.xAxis=np.arange(0,(self.numPoints)*self.timeResolution,self.timeResolution)
        self.xAxis=self.xAxis[0:-1]
        
    def showPlot(self):
        self.PO2canvas.show()
        
    def clearPlot(self):
        self.PO2AbsCurve.clear()
        self.PO2NormCurve.clear()
        self.PO2LogNormCurve.clear()
        self.PO2FFTCurve.clear()
        
    def initPlot(self,numAcquisitions):
        self.counter=-1
        self.clearPlot()
        numPoints=int(round(self.numPoints))
            
        if (self.previousTracesFlag==0):
            self.storedData=np.zeros((numPoints,int(numAcquisitions)))
            self.normData=np.zeros((numPoints,int(numAcquisitions)))
            self.LogData=np.zeros((numPoints,int(numAcquisitions)))
            self.FFTData=np.zeros((int(numPoints/2),int(numAcquisitions)))
            

class CurrentLineViewer(Queue.Queue):

    def __init__(self,data):
        
        Queue.Queue.__init__(self,2)
        
        self.currentImv = pg.ImageView(None, 'Line Viewer')
        self.currentImv.setWindowTitle('Line Viewer')
        self.currentScene=self.currentImv.scene
        self.data=data
        
        self.currentImv.show()
        self.currentImi=self.currentImv.getImageItem()
        self.setAngio(self.data)
        self.currentlines = []

        print("CurrentLineViewer created!")

    def setAngio(self,data):
        self.currentImv.setImage(data,pos=(0,0),scale=(1,1))

    def displayCurrentLine(self,x_center,y_center,angleLine,length):
        print('currentLines number: '+str(len(self.currentlines)))
        if len(self.currentlines)>0:
            for line in self.currentlines:
                print("killing lines")
                self.currentImv.removeItem(line)
        self.currentlines=[]
        x1=x_center+math.cos(angleLine)*length/2;
        y1=y_center+math.sin(angleLine)*length/2;        
        x2=x_center-math.cos(angleLine)*length/2;
        y2=y_center-math.sin(angleLine)*length/2;
        self.currentlines.append(LineScanROI([x2, y2], [x1, 2*y2-y1], width=1, pen=pg.mkPen('y',width=7)))
        self.currentImv.addItem(self.currentlines[-1])


class ChannelViewer(Queue.Queue):

    def __init__(self, name,windowYPosition):
        
        Queue.Queue.__init__(self,2)
        
        self.imv = pg.ImageView(None, name)
        self.imv.setWindowTitle(name)
        self.Scene=self.imv.scene
        self.lines = []
        self.rect = [] 
        self.points = []
        self.rectFlag = []

        #self.generateRandomLine()
        self.linescan_not_displayed=0
        self.lineSelected=-1
        self.rectSelected=-1
        self.setTestImage()
        #self.generateAutoLines()
        #self.displayLogo()
        self.intensityCurve =[]
        self.intensityFlag=0
        
        # display and set on_click callback
        self.shift_display = 0  
        self.shift_display_live = 0  
        self.displayLines()
        #self.imv.move(1200, windowYPosition)
        #self.imv.resize(650, 450)
        self.imv.show()
        self.imi=self.imv.getImageItem()
        self.generatePointFlag=False
        self.generateLineFromPointFlag=False
        self.lineScanFlag=False
        self.orthogonalLines=False
        self.Scene.sigMouseClicked.connect(self.onClick)
        self.counterLineFromPoint=0
        self.pointsForLine=[]
        self.lineFromPoint_x0=[]
        self.lineFromPoint_x1=[]
        self.lineFromPoint_y0=[]
        self.lineFromPoint_y1=[]
        #self.Scene.sigMouseClicked.connect(self.mouseMoved)
        self.levelFlag=True
        self.averageOn=False
        self.scanningType='SawTooth'
        self.parula=np.array([[0.2081,0.21056,0.21215,0.21235,0.21074,0.20655,0.19887,0.1865,0.16753,0.1403,0.1025,0.059133,0.022355,0.0073394,0.0054788,0.010618,0.019227,0.029709,0.040964,0.051176,0.059745,0.066809,0.072267,0.076391,0.078773,0.079421,0.07783,0.0734,0.066412,0.057352,0.047336,0.037745,0.030809,0.0265,0.024512,0.02367,0.023155,0.022839,0.023139,0.025373,0.030697,0.040027,0.052655,0.067673,0.0843,0.10242,0.12178,0.14229,0.16397,0.18679,0.21078,0.236,0.26247,0.29006,0.31872,0.34817,0.37811,0.40817,0.43785,0.46686,0.49509,0.52245,0.54884,0.57447,0.59945,0.62365,0.6473,0.67042,0.6931,0.71528,0.73715,0.75859,0.77983,0.80074,0.82145,0.84194,0.86228,0.88243,0.90249,0.92243,0.94216,0.96128,0.97853,0.99135,0.99777,0.99901,0.99698,0.99313,0.988,0.98224,0.97632,0.97047,0.96545,0.96165,0.95924,0.95867,0.9602,0.96384,0.96956,0.9763],[0.1663,0.18124,0.19629,0.21156,0.22718,0.24324,0.25971,0.27661,0.29462,0.31475,0.33749,0.35983,0.37858,0.39372,0.40688,0.4187,0.42967,0.44015,0.45018,0.45997,0.46961,0.47914,0.48867,0.49832,0.50821,0.51852,0.52937,0.54096,0.55331,0.56618,0.57904,0.59147,0.60302,0.6137,0.62347,0.6324,0.64065,0.64828,0.65547,0.66228,0.66887,0.67515,0.68122,0.68714,0.69283,0.69841,0.70382,0.70902,0.71411,0.719,0.72372,0.72812,0.7323,0.73612,0.73951,0.74243,0.74479,0.74659,0.7479,0.74869,0.7491,0.74919,0.74893,0.74842,0.74765,0.74673,0.7456,0.74431,0.74291,0.74136,0.73969,0.73788,0.73608,0.73428,0.73248,0.73067,0.72895,0.72743,0.72624,0.7256,0.72593,0.72806,0.73322,0.74223,0.75418,0.76719,0.78043,0.79355,0.8066,0.81964,0.83277,0.84619,0.86013,0.87476,0.89026,0.9068,0.92444,0.94316,0.96281,0.9831],[0.5292,0.55991,0.59103,0.62245,0.65426,0.68628,0.71863,0.75108,0.78396,0.81675,0.84688,0.86833,0.87932,0.88299,0.88306,0.8809,0.87756,0.87328,0.86847,0.86332,0.85782,0.8523,0.8467,0.84132,0.83625,0.83177,0.82804,0.82566,0.82425,0.82349,0.82266,0.82102,0.81796,0.8135,0.80766,0.80055,0.79247,0.78351,0.77396,0.76374,0.75307,0.74193,0.73046,0.71846,0.70617,0.69335,0.68018,0.66658,0.65266,0.63829,0.62353,0.60855,0.59326,0.57781,0.56245,0.54727,0.53254,0.51848,0.50519,0.49255,0.48065,0.46928,0.45837,0.44794,0.43798,0.42823,0.4188,0.40958,0.40057,0.39173,0.38297,0.37431,0.36575,0.3572,0.34854,0.33978,0.33087,0.3217,0.31205,0.30176,0.29041,0.27736,0.26204,0.24512,0.22885,0.21443,0.20178,0.19012,0.17937,0.16905,0.15901,0.14892,0.13862,0.12775,0.11655,0.10499,0.093118,0.080927,0.068094,0.0538]])

    def initPO2Calculation(self,calcFlag):
        self.estimatePO2=calcFlag
        if (self.estimatePO2==1):
            self.lowBoundary=10
            self.upperBoundary=80
        else:
            self.lowBoundary=17
            self.upperBoundary=150            

    def setAutoLevels(self):
        self.imv.autoLevels()
        
    def setAutoRange(self):
        self.imv.autoRange()       
        
    def setGalvoController(self,galvo_controller):
        self.galvo_controller=galvo_controller
        
    def displayLogo(self):
        logo = np.asarray(Image.open('C:\git-projects\multiphoton\liom_logo.png'))
        logo.setflags(write=1)
        logo=logo.transpose((1,0,2))
        self.imv.setImage(logo,pos=(0,0),scale=(1,1)) 
        
    def mouseMoved(self,pos):
        print ("Image position:", self.imi.mapToView(pos))
        
    def updateGeneratePointFlag(self,flag):
        self.generatePointFlag=flag
        
    def updateGenerateLineFromPointFlag(self,flag):
        self.generateLineFromPointFlag=flag
        
    def onClick(self, event):
        posMouse=self.imi.mapFromScene(event.scenePos())
        items=self.Scene.items(event.scenePos())
        if (self.generatePointFlag):
            if event.button()==1:
                print ('generating point')
                self.createPoint(posMouse.x(), posMouse.y())
            elif event.button()==2:
                for i in items:
                    if isinstance(i,PointROI):
                        self.pointSelected=i
                        self.removeSelectedPoint()
                        print('click!')
        elif(self.generateLineFromPointFlag):
            if event.button()==1:
                self.counterLineFromPoint+=1
                if (self.counterLineFromPoint % 2):
                    print('create first point')
                    self.lineFromPoint_x0=posMouse.x()
                    self.lineFromPoint_y0=posMouse.y()
                    self.createPointForLine(self.lineFromPoint_x0,self.lineFromPoint_y0)
                else:
                    print('create second point')
                    self.lineFromPoint_x1=posMouse.x()
                    self.lineFromPoint_y1=posMouse.y()
                    self.createPointForLine(self.lineFromPoint_x1,self.lineFromPoint_y1)
                    self.createLineFromPoint(self.lineFromPoint_x0,self.lineFromPoint_y0,self.lineFromPoint_x1,self.lineFromPoint_y1)
                    self.removeAllPointsForLines()
        else:    
            for i in items:
                if isinstance(i, LineScanROI):
                    self.lineSelected=i
                    if event.button()==2:
                        self.removeSelectedLine()
                    elif event.button()==1:
                        self.CurrentLineInfo=self.getMouseSelectedLinePosition()
                        self.galvo_controller.update_linescan()
                if isinstance(i,RectROI):
                    self.rectSelected=i
                    if event.button()==2:
                        self.removeSelectedRectangle()
                    elif event.button()==1:
                        self.CurrentRectInfo=self.getMouseSelectedRectPosition()     
                if isinstance(i,PointROI):
                    self.pointSelected=i
                    if event.button()==2:
                        self.removeSelectedPoint()
                        print('click!')
                #elif event.button()==1:
                    #self.CurrentPointInfo=self.getMouseSelectedPointPosition()                    
                    
    def createPoint(self,posX,posY):
        self.points.append(PointROI([posX, posY],[5,5],pen=pg.mkPen('r',width=4.5)))
        #self.points.append(pointROI([posX, posY])) #pen=(4,9)
        self.imv.addItem(self.points[-1])
        self.getPositionPoints()
        
    def createPointForLine(self,posX,posY):
        self.pointsForLine.append(PointForLineROI([posX, posY],[5,5],pen=pg.mkPen('r',width=4.5)))
        #self.points.append(pointROI([posX, posY])) #pen=(4,9)
        self.imv.addItem(self.pointsForLine[-1])
        
    def createLineFromPoint(self,x0,y0,x1,y1):
        self.resetLines()        
        self.lines.append(LineScanROI([x1, y1], [x0, 2*y1-y0], width=1, pen=pg.mkPen('y',width=7)))
        self.imv.addItem(self.lines[-1])
        self.displayLines()
        
    def addLines(self):
        self.resetLines()        
        x1=random.uniform(1, 256)
        y1=random.uniform(1, 256)
        angleLine=random.uniform(0,math.pi)
        length=60;
        x2=x1+math.cos(angleLine)*length;
        y2=y1+math.sin(angleLine)*length;
        self.lines.append(LineScanROI([x1, y1], [x2, y2], width=1, pen=pg.mkPen('y',width=7)))
        self.imv.addItem(self.lines[-1])
        self.displayLines()
        
    def getPositionPoints(self):
        x=[]
        y=[]
        if len(self.points)>0:
            for point in self.points:
                tempPos=point.pos()
                x.append(tempPos[1])
                y.append(tempPos[0])
        else:
            print('no points on Image')
        return x,y
        
    def displayPoints(self):
        if len(self.points)>0:
            for point in self.points:
                self.imv.addItem(point)
                
    def displayLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.addItem(line)
            #self.imv.show()

    def removeLastPoints(self,number):
        numPoints=len(self.points)
        pointsToDelete=np.linspace(numPoints-number,numPoints-1,number)
        pointsToDelete=np.flip(pointsToDelete)
        print (pointsToDelete)
        for i in pointsToDelete:
            print(i)
            del self.points[int(i)]
        self.displayPoints()

    def createRectangle(self,nx,ny):
        width=round(nx/5)
        height=round(ny/5)
        xOrigin=round(nx/2)-round(width/2)
        yOrigin=round(ny/2)-round(height/2)
        self.rect.append(RectROI([xOrigin,yOrigin], size=[width,height], angle=0.0, invertible=False, maxBounds=None, snapSize=1.0, scaleSnap=False, translateSnap=False, rotateSnap=False, parent=None, pen='y', movable=True, removable=False))
        self.rect[-1].addScaleHandle(pos=[1,1],center=[0,0])
        self.rectFlag.append(0)
        #self.rect[-1].addRotateHandle(pos=[0,1],center=[0.5,0.5])
    def showWindow(self):
        self.imv.show()
            

                
    def displayRectangles(self):
        if len(self.rect)>0:
            for selRect in self.rect:
                self.imv.addItem(selRect)        
                
    def resetLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.removeItem(line)
                
    def removeAllLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.removeItem(line)
        self.lines=[]
                
    def resetRect(self):
        counter=0
        if len(self.rect)>0:
            for selRect in self.rect:
                self.imv.removeItem(selRect)
                del self.rect[counter]
                counter=counter+1
                
    def resetPoint(self):
        if len(self.points)>0:
            for point in self.points:
                self.imv.removeItem(point)
                
    def removeAllPoints(self):
        if len(self.points)>0:
            for point in self.points:
                self.imv.removeItem(point)
        self.points=[]

    def removeAllPointsForLines(self):
        if len(self.pointsForLine)>0:
            for point in self.pointsForLine:
                self.imv.removeItem(point)
        self.pointsForLine=[]
                
    def highlightLine(self,selectedLine):
        if len(self.lines)>0:
            for line in self.lines:
                    if line==selectedLine:
                        line.setPen('b',width=7)
                    else:
                        line.setPen('y',width=7)
                        
    def highlightPoint(self,selectedPoint):
        counter=0
        if len(self.points)>0:
            for selPoint in self.points:
                if counter==selectedPoint:
                    selPoint.setPen('w',width=4.5)
                #else:
                    #selPoint.setPen('r',width=4.5)
                counter=counter+1

    def highlightPointAll(self,selectedPoint):
        counter=0
        if len(self.points)>0:
            for selPoint in self.points:
                if counter==selectedPoint:
                    selPoint.setPen('y',width=4.5)
                else:
                    selPoint.setPen('r',width=4.5)
                counter=counter+1
                        
    def characPoint(self,selectedPoint,val):
        counter=0
        val=(val-self.lowBoundary)/(self.upperBoundary-self.lowBoundary)*99
        val=int(np.round(val))
        if (val<0):
            val=0
        if (val>98):
            val=99
        print('val'+str(val))
        r=self.parula[0,val]*255
        g=self.parula[1,val]*255
        b=self.parula[2,val]*255
            
        if len(self.points)>0:
            for selPoint in self.points:
                if counter==selectedPoint:
                    selPoint.setPen((r,g,b),width=4.5)
                elif counter>selectedPoint:
                    selPoint.setPen('r',width=4.5)
                counter=counter+1    

    def highlightRect(self,selectedRect):
        if len(self.rect)>0:
            for selRec in self.rect:
                    if selRec==selectedRect:
                        selRec.setPen('b',width=7)
                    else:
                        selRec.setPen('y',width=7)
        
    def removeSelectedRectangle(self):
        self.resetRect()
        counter=0
        delRec=-1
        if len(self.rect)>0:
            for selRec in self.rect:
                if selRec==self.rectSelected:
                    delRec=counter
                counter=counter+1
                if delRec!=-1:
                    del self.rect[delRec]                      
            self.displayRectangles()
        else:
            print('not enough rectangles to delete')

    def removeSelectedPoint(self):
        self.resetPoint()
        counter=0
        delpoint=-1
        if len(self.points)>0:
            for point in self.points:
                if point==self.pointSelected:
                    delpoint=counter
                    del self.points[delpoint]
                counter=counter+1                    
            self.displayPoints()
        else:
            print('not enough points to delete')

    def removeSelectedLine(self):
        flagOrthogonal=self.orthogonalLines
        self.resetLines()
        indexToDelete=self.lines.index(self.lineSelected)
        if len(self.lines)>0:
            if flagOrthogonal:
                
                if np.isin(indexToDelete, self.indicesOrthogonalLines):
                    linestodelete=[indexToDelete-1,indexToDelete]
                else:
                    linestodelete=[indexToDelete,indexToDelete+1]
                linestodelete.reverse()
                for i in linestodelete:
                    del self.lines[i]
                
            else:
                del self.lines[indexToDelete]
                           
            self.displayLines()
        else:
            print('not enough lines to delete')
            
    def shiftLines(self,shiftX,shiftY):
        numberOfLines=len(self.lines)
        x_0=np.zeros(numberOfLines)
        y_0=np.zeros(numberOfLines)
        x_1=np.zeros(numberOfLines)
        y_1=np.zeros(numberOfLines)
                
        vec=np.arange(numberOfLines)
        vec=np.flip(vec)
        counter=0
        if numberOfLines>0:
            for i in vec:
                x_e, y_e=self.lines[i].pos()
                length,width=self.lines[i].size()
                alpha_e=self.lines[i].angle()*math.pi/180
                x_0[i]=x_e-width/2*math.sin(alpha_e)+shiftX
                y_0[i]=y_e+width/2*math.cos(alpha_e)+shiftY
                delta_x= length * math.cos(alpha_e)
                delta_y= length * math.sin(alpha_e)
                x_1[i]=x_0[i]+delta_x
                y_1[i]=y_0[i]+delta_y
            self.removeAllLines()
            print(self.lines)
            self.lines=[]
            self.displayLines()
            for i in vec:
                self.lines.append(LineScanROI([x_1[counter], y_1[counter]], [x_0[counter], 2*y_1[counter]-y_0[counter]], width=1, pen=pg.mkPen('y',width=7)))
                counter=counter+1
            self.displayLines()
        else:
            print('not enough lines to delete')

        
    def getScanningType(self,typeOfScan):
        self.scanningType=typeOfScan
                
    def generateRandomLine(self):
        x1=random.uniform(1, 256)
        y1=random.uniform(1, 256)
        angleLine=random.uniform(0,math.pi)
        length=60;
        x2=x1+math.cos(angleLine)*length;
        y2=y1+math.sin(angleLine)*length;
        self.lines.append(LineScanROI([x1, y1], [x2, y2], width=1, pen=pg.mkPen('y',width=7)))

    def getCurrentLinePosition(self):
        x_e, y_e=self.region.pos()                  #coordinates of the lower left corner edge
        length,width=self.region.size()             #length and width of the line
        alpha_e=self.region.angle()*math.pi/180     #angle in radiants of the line
        x_0=x_e+width/2*math.sin(alpha_e)           #coordinate transform to obtain origin of line
        y_0=y_e+width/2*math.cos(alpha_e)        
        delta_x= length * math.cos(alpha_e)
        delta_y= length * math.sin(alpha_e)      
        x_1=x_0+delta_x
        y_1=y_0+delta_y
        return x_0, y_0, x_1, y_1, length
  
        
    def getAngleAndCenterSelectedLinePosition(self,lineNumber):
        x_e, y_e=self.lines[lineNumber].pos() #coordinates of the lower left corner edge
        length,width=self.lines[lineNumber].size()
        alpha_e=self.lines[lineNumber].angle()*math.pi/180.0
        x_e=x_e+width/2*math.sin(alpha_e)           #coordinate transform to obtain origin of line
        y_e=y_e+width/2*math.cos(alpha_e)         
        x_center=x_e+length/2*math.cos(alpha_e)
        y_center=y_e+length/2*math.sin(alpha_e)
       
        return x_center, y_center, alpha_e, length
        
    def forceLength(self,length):
        flagOrthogonal=self.orthogonalLines
        if flagOrthogonal:
            self.removeOrthogonalLines()
        counter=0
        numberOfLines=len(self.lines)
        x_e=np.zeros(numberOfLines)
        y_e=np.zeros(numberOfLines)
        alpha_e=np.zeros(numberOfLines)
        self.indicesOrthogonalLines=np.zeros(numberOfLines)
        if numberOfLines>0:
            for counter in range(numberOfLines):
                x_e[counter], y_e[counter], alpha_e[counter],tmp=self.getAngleAndCenterSelectedLinePosition(counter)
            self.displayLines()
            self.removeAllLines()
            for i in range(numberOfLines):
                self.generateLineFixedLength(x_e[i], y_e[i], alpha_e[i],length)
            self.displayLines()
        else:
            print('not enough lines to delete')
        if flagOrthogonal:
            self.generateOrthogonalLines()
        
            
    def generateOrthogonalLines(self):
        self.orthogonalLines=True
        self.resetLines()
        counter=0
        numberOfLines=len(self.lines)
        x_e=np.zeros(numberOfLines)
        y_e=np.zeros(numberOfLines)
        lengthVector=np.zeros(numberOfLines)
        alpha_e=np.zeros(numberOfLines)
        self.indicesOrthogonalLines=np.zeros(numberOfLines)
        if numberOfLines>0:
            for counter in range(numberOfLines):
                x_e[counter], y_e[counter], alpha_e[counter], lengthVector[counter]=self.getAngleAndCenterSelectedLinePosition(0)
                del self.lines[0]                
            self.displayLines()
            self.resetLines()
            counterOrtho=0
            for i in range(numberOfLines):
                self.generateLineFixedLength(x_e[i], y_e[i], alpha_e[i],lengthVector[i])
                self.generateLineFixedLength(x_e[i], y_e[i], alpha_e[i]+math.pi/2,lengthVector[i])
                counterOrtho=counterOrtho+1
            self.displayLines()
            self.indicesOrthogonalLines=np.arange(1,numberOfLines*2,2)
        else:
            print('not enough lines to delete')
            
    def removeOrthogonalLines(self):
        self.orthogonalLines=False
        self.resetLines()
        numberOfLines=len(self.lines)
        indicesToReview=range(numberOfLines)
        indicesToReview.reverse()
        if numberOfLines>0:
            for counter in indicesToReview:
                if counter in self.indicesOrthogonalLines:
                    del self.lines[counter]             
            self.displayLines()
        else:
            print('not enough lines to delete')
        self.indicesOrthogonalLines=[]
        
        
    def generateLineFixedLength(self,x_center,y_center,angleLine,length):
        x1=x_center+math.cos(angleLine)*length/2;
        y1=y_center+math.sin(angleLine)*length/2;        
        x2=x_center-math.cos(angleLine)*length/2;
        y2=y_center-math.sin(angleLine)*length/2;
        self.lines.append(LineScanROI([x2, y2], [x1, 2*y2-y1], width=1, pen=pg.mkPen('y',width=7)))



        
    def generateAutoLines(self, 
                          scales=3, 
                          diam=10.0, 
                          length=25.0, 
                          tolerance=0.1):
        
        self.resetLines()
        image=self.getCurrentImage()
        image=(image-image.min())/np.max(image-image.min())
        sl=ScanLines(image*255.0)


        #binary map
        sl.CalcBinaryMap(scales=scales)
        binmap=sl.GetOutputBinaryMap()
                
        #graphed skeleton
        sl.CalcGraphFromSeg()
        graph=sl.GetOutputGraph()
        print('Topological graph is created!')
        
        #potential linescans
        sl.CalcLines(diam=diam, length=length, tolerance=tolerance)
        lines=sl.GetOutputLines()
        
        self.lines=[]
                
        for i in lines:
            
            x1=i[1,0]
            y1=i[1,1]
            
            x2=i[0,0]
            y2=i[0,1]

            cond1=x1>20
            cond2=x2>20
            cond3=x1<512
            cond4=x2<512

            if (cond1 and cond2 and cond3 and cond4):
                self.lines.append(LineScanROI([y1, x1], [y2, 2*x1-x2], width=1, pen=pg.mkPen('y',width=7)))
            #self.lines.append(LineScanROI([x1, y1], [x2, 2*y1-y2], width=1, pen=pg.mkPen('y',width=3)))
        
        print('Scan lines are created!')
  
    def getCurrentImage(self):
    
        a=np.array(self.imv.image).astype('float')
        print(a.shape)
        
        return a
        
    def setTestImage(self, path='C:/Users/LiomW17/Desktop/Rafat_documents/vasc_seg/data/im1.mat'):
        
        raw=sio.loadmat(path)['im']   
        s=250
        raw=raw[s,:,:]
        self.imv.setImage(-raw,pos=(0,0),scale=(1,1))
        
    def setLineScanFlag(self,status):
        self.lineScanFlag=status
    
    @pyqtSlot(int)
    def move_offset_display(self, shift_display):
        print('shift display value: '+str(shift_display))
        self.shift_display = shift_display   
    
    @pyqtSlot(int)
    def move_offset_display_live(self, shift_display_live):
        print('shift display live value: '+str(shift_display_live))
        self.shift_display_live = shift_display_live
    
    def createAveragedDataBuffer(self,sizeBuffer,sizeX,sizeY):
        print('created!')
        self.avData=np.zeros((sizeBuffer,sizeY,sizeX))
        self.counter=0
        self.sizeBuffer=sizeBuffer
        
    def checkAverageFlag(self,flag,sizeBuffer,sizeX,sizeY):
        self.averageOn=flag
        self.createAveragedDataBuffer(sizeBuffer,sizeX,sizeY)
        
    def plotIntensityFlag(self,intensityFlag):
        print('intensity Flag set!')
        self.intensityFlag=intensityFlag
        self.intensityCurve=[]
        if intensityFlag:
            self.intensityCurvePlot = pg.PlotWidget(None)
            self.intensityCurvePlot.setWindowTitle('Intensity viewer')
            self.intensityCurvePlot.plot(title='Intensity viewer')
            self.intensityCurvePlot.setTitle('Intensity over frames')
            self.intensityCurvePlot.setLabel('left', text='Intensity')
            self.intensityCurvePlot.setLabel('bottom', text='Acqusitions')
            self.intensityCurvePlot.show()
            
    def update(self):
        try:
            data = self.get(False)
            if self.intensityFlag:
                tempAv=-np.mean(data)
                print(tempAv)
                self.intensityCurve.append(tempAv)
                self.intensityCurvePlot.plot(self.intensityCurve,pen=pg.mkPen('w', width=3))
            if (self.averageOn and not(self.lineScanFlag)):
                print('averaging on! '+str(self.counter))


                self.avData[self.counter,:,:]=data
                self.counter=(self.counter+1)%self.sizeBuffer
                sizeArray = self.avData.shape
                #tmp=self.avData.reshape((sizeArray[0]*sizeArray[1],sizeArray[2]))
                tmp=np.mean(self.avData,axis=0)
                self.imv.setImage(-tmp,autoRange=False,autoLevels=self.levelFlag,pos=(0,0),scale=(1,1))
                self.imi=self.imv.getImageItem()
            else:
                if (self.lineScanFlag):
                    #inverting reverse path
                    sizeArray = data.shape
                    rowIndicesToInvert=np.arange(0,sizeArray[0],2)
                    colIndices=np.arange(0,sizeArray[1],1)
                    colIndicesInvert=np.arange(sizeArray[1]-1,-1,-1)
                    tmp=data[rowIndicesToInvert,:]
                    tmp[:,colIndices]=tmp[:,colIndicesInvert]
                    data[rowIndicesToInvert,:]=tmp
                    #shifting of lines:
                    rowIndicesToRoll=np.arange(0,sizeArray[0],2)
                    colIndices=np.arange(0,sizeArray[1],1)
                    colIndicesRoll=np.roll(colIndices,self.shift_display)
                    tmp=data[rowIndicesToRoll,:]
                    tmp[:,colIndices]=tmp[:,colIndicesRoll]
                    data[rowIndicesToRoll,:]=tmp
                    data=data.transpose()
                if (self.scanningType=='Triangular'):
                    #inverting reverse path
                    sizeArray = data.shape
                    rowIndicesToInvert=np.arange(0,sizeArray[0],2)
                    colIndices=np.arange(0,sizeArray[1],1)
                    colIndicesInvert=np.arange(sizeArray[1]-1,-1,-1)
                    tmp=data[rowIndicesToInvert,:]
                    tmp[:,colIndices]=tmp[:,colIndicesInvert]
                    data[rowIndicesToInvert,:]=tmp
                    #shifting of lines:
                    rowIndicesToRoll=np.arange(0,sizeArray[0],2)
                    colIndices=np.arange(0,sizeArray[1],1)
                    colIndicesRoll=np.roll(colIndices,self.shift_display_live)
                    tmp=data[rowIndicesToRoll,:]
                    tmp[:,colIndices]=tmp[:,colIndicesRoll]
                    data[rowIndicesToRoll,:]=tmp
                    data=np.flip(data,axis=1)
                    
                self.imv.setImage(-data,autoRange=False,autoLevels=self.levelFlag,pos=(0,0),scale=(1,1))
                self.imi=self.imv.getImageItem()

            #- pour direct
        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass
        
    def changeLevelFlag(self,flag):
        self.levelFlag=flag
        
    def toggleLinearSelection(self):
        if self.linescan_not_displayed:
            self.displayLines()
            self.linescan_not_displayed=0
        else:
            self.resetLines()
            self.linescan_not_displayed=1
            
    def getSelectedLinePosition(self, lineNumber):
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
      
        
         
         #angle in radiants of the line

#     def getMouseSelectedLinePosition(self, item):
#         x_e, y_e=item.pos()
#         length,width=item.size()
#         alpha_e=item.angle()*math.pi/180     #angle in radiants of the line
#         x_0=x_e-width/2*math.sin(alpha_e)           #coordinate transform to obtain origin of line
#         y_0=y_e+width/2*math.cos(alpha_e)        
#         delta_x= length * math.cos(alpha_e)
#         delta_y= length * math.sin(alpha_e)      
#         x_1=x_0+delta_x
#         y_1=y_0+delta_y
#         return x_0, y_0, x_1, y_1, length 

    def getMouseSelectedLinePosition(self):
        if (self.lineSelected != -1):
            item=self.lineSelected
            x_e, y_e=item.pos()
            self.highlightLine(item)            
            length,width=item.size()
            alpha_e=item.angle()*math.pi/180     #angle in radiants of the line
            x_0=x_e-width/2*math.sin(alpha_e)           #coordinate transform to obtain origin of line
            y_0=y_e+width/2*math.cos(alpha_e)        
            delta_x= length * math.cos(alpha_e)
            delta_y= length * math.sin(alpha_e)      
            x_1=x_0+delta_x
            y_1=y_0+delta_y
        else:
            print("no lines selected!")
            x_0=0
            y_0=0
            x_1=0
            y_1=0
            length=0
        return x_0, y_0, x_1, y_1, length 

    def getMouseSelectedRectPosition(self):
        if (self.rectSelected != -1):
            item=self.rectSelected
            x_Origin, y_Origin=item.pos()
            self.highlightRect(item)            
            width,height=item.size()
        else:
            print("no rect selected!")
            x_Origin=0
            y_Origin=0
            height=0
            width=0
        return x_Origin, y_Origin, width, height

    def removeMouseSelectedLine(self, item):
        self.imv.removeItem(item)
            
    
    
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