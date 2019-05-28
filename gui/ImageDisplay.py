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

class PointROI(pg.graphicsItems.ROI.EllipseROI):

    def __init__(self, pos, size, **args):
        pg.graphicsItems.ROI.ROI.__init__(self, pos, size, **args)
        self.aspectLocked = True


class po2Viewer(Queue.Queue):

    def __init__(self, name):
        
        Queue.Queue.__init__(self,2)
        
        self.po2plot = pg.PlotWidget(None)
        self.po2plot.setWindowTitle(name)
        self.po2plot.plot(title=name)
        self.po2plot.setTitle(name)
        self.po2plot.setLabel('left', text='Intensity')
        self.po2plot.setLabel('bottom', text='Time')
        
        self.po2plot.resize(650, 450)
        self.po2plot.move(1200, 500)
        
    def showPlot(self):
        self.po2plot.show()
        
    def update(self):
        try:
            print('getting data...')
            data = self.get(False)
            #averageDecayCurve=np.mean(data,2)
            #x=np.linspace(0,data/(1e6),data.shape[0])
            self.po2plot.plot(data)
            print('data plotted!')
            #- pour direct
        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass        
        #self.Scene.sigMouseClicked.connect(self.mouseMove   
     

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
        #self.setTestImage()
        #self.generateAutoLines()
        self.displayLogo()
        # display and set on_click callback
        self.shift_display = 0  
        self.displayLines()
        self.imv.move(1200, windowYPosition)
        self.imv.resize(650, 450)
        self.imv.show()
        self.imi=self.imv.getImageItem()
        self.generatePointFlag=False
        self.lineScanFlag=False
        self.Scene.sigMouseClicked.connect(self.onClick)
        #self.Scene.sigMouseClicked.connect(self.mouseMoved)
        
    def displayLogo(self):
        logo = np.asarray(Image.open('C:\git-projects\multiphoton\liom_logo.png'))
        logo.setflags(write=1)
        logo=logo.transpose((1,0,2))
        self.imv.setImage(logo) 
        
    def mouseMoved(self,pos):
        print "Image position:", self.imi.mapToView(pos)
        
    def updateGeneratePointFlag(self,flag):
        self.generatePointFlag=flag
        
    def onClick(self, event):
        posMouse=self.imi.mapFromScene(event.scenePos())
        items=self.Scene.items(event.scenePos())
        if (self.generatePointFlag):
            if event.button()==1:
                print 'generating point'
                self.createPoint(posMouse.x(), posMouse.y())
            elif event.button()==2:
                for i in items:
                    if isinstance(i,PointROI):
                        self.pointSelected=i
                        self.removeSelectedPoint()
                        print('click!')
        else:    
            for i in items:
                if isinstance(i, LineScanROI):
                    self.lineSelected=i
                    if event.button()==2:
                        self.removeSelectedLine()
                    elif event.button()==1:
                        self.CurrentLineInfo=self.getMouseSelectedLinePosition()
                if isinstance(i,pg.ROI):
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
        
    def getPositionPoints(self):
        x=[]
        y=[]
        if len(self.points)>0:
            for point in self.points:
                tempPos=point.pos()
                x.append(tempPos[0])
                y.append(tempPos[1])
        else:
            print('no points on Image')
        return x,y
        
    def displayPoints(self):
        if len(self.points)>0:
            for point in self.points:
                self.imv.addItem(point)

    def removeLastPoints(self,number):
        numPoints=len(self.points)
        pointsToDelete=np.linspace(numPoints-number,numPoints-1,number)
        pointsToDelete=np.flip(pointsToDelete)
        print pointsToDelete
        for i in pointsToDelete:
            print(i)
            del self.points[int(i)]
        self.displayPoints()

    def createRectangle(self,nx,ny):
        width=round(nx/5)
        height=round(ny/5)
        xOrigin=round(nx/2)-round(width/2)
        yOrigin=round(ny/2)-round(height/2)
        self.rect.append(pg.ROI([xOrigin,yOrigin], size=[width,height], angle=0.0, invertible=False, maxBounds=None, snapSize=1.0, scaleSnap=False, translateSnap=False, rotateSnap=False, parent=None, pen='y', movable=True, removable=False))
        self.rect[-1].addScaleHandle(pos=[1,1],center=[0,0])
        self.rectFlag.append(0)
        #self.rect[-1].addRotateHandle(pos=[0,1],center=[0.5,0.5])
    def showWindow(self):
        self.imv.show()
            
    def displayLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.addItem(line)
                
    def displayRectangles(self):
        if len(self.rect)>0:
            for selRect in self.rect:
                self.imv.addItem(selRect)        
                
    def resetLines(self):
        if len(self.lines)>0:
            for line in self.lines:
                self.imv.removeItem(line)
                
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
                
    def highlightLine(self,selectedLine):
        if len(self.lines)>0:
            for line in self.lines:
                    if line==selectedLine:
                        line.setPen('b')
                    else:
                        line.setPen('y')

    def highlightRect(self,selectedRect):
        if len(self.rect)>0:
            for selRec in self.rect:
                    if selRec==selectedRect:
                        selRec.setPen('b')
                    else:
                        selRec.setPen('y')

    def addLines(self):
        self.resetLines()        
        x1=random.uniform(1, 256)
        y1=random.uniform(1, 256)
        angleLine=random.uniform(0,math.pi)
        length=60;
        x2=x1+math.cos(angleLine)*length;
        y2=y1+math.sin(angleLine)*length;
        self.lines.append(LineScanROI([x1, y1], [x2, y2], width=1, pen=pg.mkPen('y',width=3)))
        self.displayLines()
        
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
        self.resetLines()
        counter=0
        selLine=-1
        if len(self.lines)>0:
            for line in self.lines:
                    if line==self.lineSelected:
                        selLine=counter
                    counter=counter+1
                    if selLine!=-1:
                        del self.lines[selLine]        
            self.displayLines()
        else:
            print('not enough lines to delete')
            
    def generateRandomLine(self):
        x1=random.uniform(1, 256)
        y1=random.uniform(1, 256)
        angleLine=random.uniform(0,math.pi)
        length=60;
        x2=x1+math.cos(angleLine)*length;
        y2=y1+math.sin(angleLine)*length;
        self.lines.append(LineScanROI([x1, y1], [x2, y2], width=1, pen=pg.mkPen('y',width=3)))
        
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
            
            x1=i[1,0]
            y1=i[1,1]
            
            x2=i[0,0]
            y2=i[0,1]

            self.lines.append(LineScanROI([x1, y1], [x2, 2*y1-y2], width=1, pen=pg.mkPen('y',width=3)))
 
  
    def getCurrentImage(self):
        return np.array(self.imv.image).astype('float')
        
    def setTestImage(self, path='C:/git-projects/twophoton/vasc_seg/data/im1.mat'):
        
        raw=sio.loadmat(path)['im']   
        s=250
        raw=raw[s,:,:]
        self.imv.setImage(-raw)
        
    def setLineScanFlag(self,status):
        self.lineScanFlag=status
    
    @pyqtSlot(int)
    def move_offset_display(self, shift_display):
        self.shift_display = shift_display   
    
    def update(self):
        try:
            data = self.get(False)
            
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
            self.imv.setImage(-data)
            self.imi=self.imv.getImageItem()

            #- pour direct
        except Queue.Empty:
            # Ignore and preserve previous state of display
            pass
        
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