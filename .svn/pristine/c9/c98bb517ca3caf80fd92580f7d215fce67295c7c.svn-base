# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 11:45:17 2015

@author: flesage
"""

import Queue
import threading
import time

class TT(Queue.Queue):
    def __init__(self):     
        Queue.Queue.__init__(self,256)
    
        # Thread to display so that the puts in queue are very fast
        self.t1 = threading.Thread(target=self.display)
        self.started = True
        self.t1.start()

    def display(self):
        # This is blocking which is ok 
        while self.started:
            try:                
                print 'Getting'
                data = self.get(True,2)
                print 'Got data' + data
            except Queue.Empty:
                print 'Got no data'
        
    def close(self):
        print 'Thread stopped'
        self.started = False
        self.t1.join()
        
if __name__ == '__main__':
    a=TT()
    time.sleep(6)
    time.sleep(1)

    a.close()
    