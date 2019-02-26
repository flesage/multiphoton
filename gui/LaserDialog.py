'''
Created on 28 janv. 2019

@author: LiomW17
'''

from datetime import date
import time
import math
import os
from main import config
from PyQt5 import QtGui
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog, QDialog
from base import liomio
import numpy as np
import icons_rc
from base.liomio import DataSaver
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import posixpath
import ImageDisplay as imdisp

class LaserDialog(QDialog):

    def __init__(self):
        QDialog.__init__(self)
        basepath= os.path.join(os.path.dirname(__file__))
        uic.loadUi(os.path.join(basepath,"laser_exit.ui"), self)
            
            
    def setMaitai(self,maitai):
        self.maitai = maitai
        
    def turn_maitai_off(self):
            print 'Turning laser off...'
            self.maitai.CloseLaser() 
            self.maitai_off = True
            self.maitai.SHUTTERshut() 
        
        