# -*- coding: utf-8 -*-
"""
Created on Thu Oct 15 11:37:28 2015

@author: LiomW17
"""

import pyqtgraph as pg
from base import imgenerator
from base import liomio
from gui import ImageDisplay


if __name__ == '__main__':
    app = pg.mkQApp()
    #app = QtGui.QApplication(sys.argv)

    gen=imgenerator.RandomImage16Bits(30,1024,1024)
    viewer=ImageDisplay.OCTViewer('pg viewer')
    gen.setDataConsumer(viewer,False,0)
    #saver = liomio.DataSaver('/Users/flesage/Desktop/test.hd5')
    #gen.setDataConsumer(saver,True,0)
    #saver.startSaving()

    timer = pg.QtCore.QTimer()
    timer.timeout.connect(viewer.update)
    timer.start(0)
    gen.start()
    app.exec_()
    timer.stop()
    #saver.stopSaving()
    gen.stop()

