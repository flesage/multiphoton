# -*- coding: utf-8 -*-
"""
Created on Sat Oct 24 12:07:44 2015

@author: flesage
"""
import threading
import Queue
import numpy as np
import time

class RandomImage16Bits():
    def __init__(self,rate,nx,ny):     
        
        self.nx = nx
        self.ny = ny
        self.delay = 1.0/rate
        self.consumers = []
        
    def start(self):
        # Thread to display so that the puts in queue are very fast
        self.t1 = threading.Thread(target=self.generate)
        self.started = True
        self.t1.start()
        
    def setDataConsumer(self,consumer, wait, channel):
        
        # Consumers list contains triplet of consumer, push method and channel to push
        self.consumers.append(consumer)
        self.consumers.append(wait)
        self.consumers.append(channel)
        
    def generate(self):   
        while self.started == True:
            
            # Generate random frame
            data=np.random.random((self.nx,self.ny))*4096
            for ic in range(0,len(self.consumers),3):                
            # Push
                try:
                    self.consumers[ic].put(data.astype('uint16'),self.consumers[ic+1]) 
                except Queue.Full:
                    pass
            time.sleep(self.delay)
            
    def stop(self):   
        self.started = False
        self.t1.join()