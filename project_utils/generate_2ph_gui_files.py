# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 14:46:40 2015

@author: flesage
"""

import os
import sys
import platform

if platform.system() == 'Darwin':
    anaconda_path = '/Users/flesage/anaconda/bin'
else:
    anaconda_path = 'C:\\Users\\flesage\\Anaconda2\\Lib\\site-packages\\PyQt5'
    anaconda_path2 = 'C:\\Users\\flesage\\Anaconda2\\Library\\bin'

base_dir=os.path.dirname(os.path.dirname(sys.argv[0]))

# Generate uic/rc elements and move to gui folder
os.system(os.path.join(anaconda_path2,'pyrcc5.exe ')+os.path.join(base_dir,'liomacq_gui','icons.qrc')+' -o '+os.path.join(base_dir,'gui','icons_rc.py'))
