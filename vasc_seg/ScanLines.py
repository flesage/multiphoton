#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 28 10:35:20 2019

@author: rdamseh
"""

from GraphContraction import GraphContraction
import numpy as np
import networkx as nx
from util import isSklNodes, fixG
import itk

class ScanLines:
    
    def __init__(self, raw, scales=1, tolerance=.1):
       
        # read array
        self.__Im=itk.GetImageFromArray(raw.astype('float32'))
        self.__Im.UpdateOutputData()
        
        self.tolerance=tolerance
        self.scales=scales
        
        self.__VessMap=[]
        self.__BinaryMap=None
        self.__Skeleton=None
        self.__Graph=None
        self.__Lines=None

        #####
        # private
        #####

    def __GetVessImage(self, im, sigma=1):
        
        # apply hessian
        dimension = im.GetImageDimension()
        InputImageType = type(im)
        OutputPixelType = itk.UC
        OutputImageType = itk.Image[OutputPixelType, dimension]
        cast = itk.CastImageFilter[InputImageType, OutputImageType].New(im)
        hessianFilter = itk.HessianRecursiveGaussianImageFilter.New(cast.GetOutput())
        hessianFilter.SetSigma(sigma)
        hessianFilter.Update()
        
        # apply vesselness filter
        hessOutput = hessianFilter.GetOutput()
        dimension = hessOutput.GetImageDimension()
        InputImageType = type(hessOutput)
        OutputPixelType = itk.F
        OutputImageType = itk.Image[OutputPixelType, dimension]
        vess=itk.HessianToObjectnessMeasureImageFilter[InputImageType, 
                                                         OutputImageType].New(hessOutput)
        vess.Update()
        vessOutput = vess.GetOutput()    
        self.__VessMap.append(vessOutput)  
        return vess
        
    def __GetBinaryImage(self, im, sigma=1):   
        
        vess=self.__GetVessImage(im, sigma=sigma)     
        vessOutput = vess.GetOutput()    
        dimension = vessOutput.GetImageDimension()
        InputImageType = type(vessOutput)
        OutputPixelType = itk.F
        OutputImageType = itk.Image[OutputPixelType, dimension]
        vessOutput = itk.CastImageFilter[InputImageType, OutputImageType].New(vessOutput)
        
        # norm and thresholding from vessOutput
        norm=itk.NormalizeImageFilter.New(vessOutput)
        norm.Update()
        binary=itk.BinaryThresholdImageFilter.New(norm)
        binary.SetLowerThreshold(0)
        binary.Update()
                  
        return binary


    def __RemoveSmallObjects(self, binary, thr_size=75):
        
        # norm and thresholding from vessOutput
        binary=itk.BinaryThresholdImageFilter.New(binary.GetOutput())
        binary.SetLowerThreshold(1)
        binary.Update()    
        
        # get labels and remove small ones
        binaryOutput=binary.GetOutput()
        labels=itk.BinaryImageToLabelMapFilter.New(binaryOutput)
        labels.Update()
        labelsOutput=labels.GetOutput()
        nLabels=labelsOutput.GetNumberOfLabelObjects()
        
        size=[]
        for i in range(nLabels):   
            size.append(int(labelsOutput.GetNthLabelObject(i).Size()))
        
        labelsToRemove=np.where(np.array(size)<thr_size)[0]
        
        for i in labelsToRemove:
            labelsOutput.RemoveLabel(int(i+1))
            
        seg=itk.LabelMapToBinaryImageFilter.New(labelsOutput)
        seg.Update()
        
        return seg


    def __GetBinaryMap(self):
        
        sigmas=[1,3,5,7,9,13]
        fillRadius=3
        
        # get binary images at different ranks        
        binaryImages=[]       
        for i in range(self.scales):           
            binaryImages.append(self.__GetBinaryImage(self.__Im,    
                                        sigma=sigmas[i]))  
        
        # add binary maps 
        if self.scales>1:
            
            for i in range(len(binaryImages)):           
                if i>0:
                    if i==1:
                        add=itk.AddImageFilter.New(binaryImages[i-1], binaryImages[i])
                        add.Update()                
                    else:                            
                        add=itk.AddImageFilter.New(add.GetOutput(), binaryImages[i])
                        add.Update()
        else:
            
            add=binaryImages[0]
                            
       
        # remove small objects  
        refined=self.__RemoveSmallObjects(add, thr_size=75)    

        # fill holes
        refined=itk.VotingBinaryIterativeHoleFillingImageFilter.New(refined.GetOutput());
        refined.SetRadius(fillRadius)
        refined.Update()
        
        self.__BinaryMap=refined.GetOutput()


    def __GetSkeleton(self):
        
        if self.__BinaryMap:
            
            # get skeleton
            skl=itk.BinaryThinningImageFilter.New(self.__BinaryMap)
            skl.Update()    
            
            self.Skeleton=skl.GetOutput()
    
    def __GetGraphFromSkl(self):
        return

    def __GetGraphFromSeg(self):
        
        if self.__BinaryMap is not None:
            
            binmap=itk.GetArrayFromImage(self.__BinaryMap) 
            binmap=np.swapaxes(binmap,0,1).astype('bool')
            label=np.array(binmap[None,:])
            label=np.rollaxis(label,0,3)
            
            G=GraphContraction(label=label)
            G.generateGraph()
            G.contractGraph(speed_param=.1, #[low_speed, high_speed]
                             dis_param=1,
                             deg_param=0,
                             med_param=1,
                             thr=5,
                             n_iter=10,
                             stop_thr=.01)  
            G.refineGraph(diam_refine=5)
            graph=G.G_refined  
      
            self.__Graph=graph
         
    def __GetLines(self):
        
        diam=self.diam 
        length=self.length         
        if self.__Graph:
            
            graph=self.__Graph.copy()
            
            
            # get potential lines
            tolerance=self.tolerance
            smooth_itr=1
            seg_len=4
      
            nb=[graph.neighbors(i) for i in graph.nodes()]
            nodesToRemove1=[k for k,i in enumerate(nb) if len(i)>2]
            nodesToProcess=[k for k,i in enumerate(nb) if len(i)==2]
            
            # average position
            for i in range(smooth_itr):
                ownpos=np.array([graph.node[i]['pos'] for i in nodesToProcess])
                otherpos=np.array([[ graph.node[k]['pos']  for k in graph.neighbors(i)] for i in nodesToProcess])
                posNew=(ownpos.astype('float')+otherpos[:,0,:]+otherpos[:,1,:])/3   
                for ind, node in enumerate(nodesToProcess):
                    graph.node[node]['pos']=posNew[ind]
            
            otherpos=np.array([[ graph.node[k]['pos']  for k in graph.neighbors(i)] for i in nodesToProcess])
            indNodesToRemove2=isSklNodes(posNew, otherpos, thr=tolerance)==False  
            nodesToProcess=np.array(nodesToProcess)
            nodesToRemove2=nodesToProcess[indNodesToRemove2]
            
            graph.remove_nodes_from(nodesToRemove1)
            graph.remove_nodes_from(nodesToRemove2)
            graph=fixG(graph)
                
            # get graph segments
            components=list(nx.connected_components(graph))
            components=[i for i in components if len(i)> seg_len]       
            segments=[[i for i in j if len(graph.neighbors(i))==1] for j in components]        
            segments=np.array(segments)
            
            # remove segments with big diam
            segmentsToRecall=[]
            for idx, i in enumerate(segments):
                if graph.node[i[0]]['d']<diam and graph.node[i[1]]['d']<diam:
                    segmentsToRecall.append(idx)            
            segments=segments[segmentsToRecall,:]
       
            lines=[[np.array(graph.node[j]['pos']) for j in i] for i in segments]  
            lines=np.array(lines)
            
            # retrieve with length constrains
            norm=np.linalg.norm(lines[:,0,:]-lines[:,1,:], axis=1)
            mid=np.mean(lines, axis=1)
            unitV=(lines[:,0,:]-lines[:,1,:])/norm[:,None]
            
            IndLinesToReduce=np.where(norm>length)
            LinesReduced=np.array([mid[IndLinesToReduce]-length*unitV[IndLinesToReduce]/2.0,
                                    mid[IndLinesToReduce]+length*unitV[IndLinesToReduce]/2.0])
            LinesReduced=np.swapaxes(LinesReduced,0,1)
            lines[IndLinesToReduce]=LinesReduced
       
            self.__Lines=lines
            
        
    #######
    # Public
    #######
    
    def CalcBinaryMap(self, scales=1):
        self.scales=scales
        self.__GetBinaryMap() 
        
    def CalcSkeleton(self):
        self.__GetSkeleton()
        
    def CalcGraphFromSeg(self):
        self.__GetGraphFromSeg()

    def CalcGraphFromSkl(self):
        self.__GetGraphFromSkl()
        
    def CalcLines(self, diam=5, length=15, tolerance=.1):
        self.diam=diam
        self.length=length
        self.tolerance=tolerance
        self.__GetLines()

        
    def GetOutputBinaryMap(self):
        
        if self.__BinaryMap is not None:
            
            im=itk.GetArrayFromImage(self.__BinaryMap) 
            im=np.swapaxes(im,0,1).astype('bool')
            
            return im       

    def GetOutputVessMap(self):
        
        if len(self.__VessMap)>0:
            
            vessmap=[]
            
            for i in self.__VessMap:
                
                vess=itk.GetArrayFromImage(i) 
                vessmap.append(np.swapaxes(vess,0,1).astype('float'))   
                
            return vessmap  
                    
    def GetOutputSkeleton(self):
        
        if self.Skeleton is not None:
            
            skl=itk.GetArrayFromImage(self.Skeleton) 
            skl=np.swapaxes(skl,0,1).astype('bool')
            
            return skl

    def GetOutputGraph(self):
        
        if self.__Graph is not None: return self.__Graph
 
    def GetOutputLines(self):
        
        if self.__Lines is not None: return self.__Lines


if __name__=='__main__':    
  
    pass

