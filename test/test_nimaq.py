# -*- coding: utf-8 -*-
"""
Created on Wed Oct 14 13:26:32 2015

@author: LiomW17
"""
import time
from base import liomnimaq
from gui import ImageDisplay
    
if __name__ == '__main__':    
    cam = liomnimaq.NiIMAQCamera()
    cam.open_camera('img0')
    cam.configure_for_single_grab()
    
    viewer=ImageDisplay.ChannelViewer('OCT Test')
    cam.setDataConsumer(viewer)
    
    cam.start()
    
    time.sleep(300)
    
    viewer.close()
    cam.stop()
    cam.close_camera()
    