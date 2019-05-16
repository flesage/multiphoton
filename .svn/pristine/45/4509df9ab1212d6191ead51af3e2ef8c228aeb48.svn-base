# -*- coding: utf-8 -*-
"""
Created on Tue Sep  1 13:02:20 2015

@author: flesage
"""
import numpy as np
import matplotlib.pyplot as plt

def get_poly_return(line1_VoltStart, line1_VoltEnd, n_pts_line1, line2_VoltStart, line2_VoltEnd, n_pts_line2, n_extra_pts):
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
    
def setSawToothRamp(x0,y0,xe,ye,nx,ny,n_extra,n_repeat,line_rate):

    # Fast axis is always set to be x    
    ramp_x = np.concatenate((np.linspace(x0,xe,nx),get_poly_return(x0,xe,nx,x0,xe,nx,n_extra)))
    ramp_y = np.linspace(y0,ye,ny)
    
    ramp_x=np.tile(ramp_x,[1,ny*n_repeat])
    ramp_y=np.tile(ramp_y,[n_repeat*(nx+n_extra),1]).flatten('F').reshape(1,(nx+n_extra)*ny*n_repeat)
    ramps=np.concatenate((ramp_x,ramp_y))

    
    daq_freq=ramp_x.size*line_rate
    return ramps
    
def setTriangularRamp(x0,y0,xe,ye,nx,ny,n_extra,n_repeat,line_rate):
    
    # Need a pair value for ny
    if ny % 2 == 1:
        ny = ny-1
    # Need to build double ramp here
    ramp_x = np.concatenate((np.linspace(x0,xe,nx),get_poly_return(x0,xe,nx,xe,x0,nx,n_extra),np.linspace(xe,x0,nx),get_poly_return(xe,x0,nx,x0,xe,nx,n_extra)))
    ramp_y = np.linspace(y0,ye,ny)
    
    full_x=np.tile(ramp_x,[1,ny/2*n_repeat])
    full_y=np.tile(ramp_y,[n_repeat*(nx+n_extra),1]).flatten('F').reshape(1,(nx+n_extra)*ny*n_repeat)
    ramps=np.concatenate((full_x,full_y))
    
    daq_freq=(nx+n_extra)*line_rate
    return ramps
        
if __name__ == '__main__':
    #ramp=get_poly_return(1,10,100,10,1,100,40)
    ramp=setTriangularRamp(-200,-200,200,200,10,10,40,1,100)
    
    plt.plot(np.transpose(ramp))
    plt.show()