#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 28 10:35:20 2019

@author: rdamseh
"""


from ScanLines import ScanLines
import scipy.io as sio
from matplotlib import pyplot as plt


if __name__=='__main__':    

    # read volume
    raw=sio.loadmat('C:/git-projects/twophoton/vasc_seg/data/im1.mat')['im']   
    s=250
    raw=raw[s,:,:]
     
    
    sl=ScanLines(raw)
    scales=1 # between 1 and 6
    
    #binary map
    sl.CalcBinaryMap(scales=scales)
    binmap=sl.GetOutputBinaryMap()

    #graphed skeleton
    sl.CalcGraphFromSeg()
    graph=sl.GetOutputGraph()
    
    #potential linescans
    diam=10.0 # max diam of vessels to undergo linescaning
    tolerance=.1 # between 0 and 1
    length=20.0 # max length of linescans
    sl.CalcLines(diam=diam, length=length, tolerance=tolerance)
    lines=sl.GetOutputLines()
    
    print(lines[0])
    
    plt.imshow(raw)
    for i in lines:
        plt.plot(i[:,1], i[:,0], 'ro-', linewidth=1, markersize=2)
    plt.show()
        
        
        
        
        
        
        
        
#
