
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 12:15:12 2015

@author: flesage

    Main application for OCT imaging, this uses various components that can be reused
    but here we specify the hardware.
"""

import sys
import numpy as np
#from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
import config
from base.Galvos import Galvos
from base.Galvos import Converter
from base.liomacq import AnalogInputTask
from base.liomacq import OnDemandVoltageOutTask
from base.Motors import Motorsclass
#from base.Motors import ZaberMotor
from base.Motors import ThorlabsMotor
#from base.Motors import RotationMotor
from gui.ImageDisplay import ChannelViewer, po2Viewer
from gui.GalvosController import GalvosController
from base.Maitai import Maitai
from base.liomacq import OnDemandDigitalOutTask
from gui.LaserDialog import LaserDialog

'''
from base import testeur
from base.Motors import LabjackComport
'''



if __name__ == '__main__':
    
    app = QApplication(sys.argv)

    
    # Basic control, make sure initial state is off
    shutter2ph = OnDemandDigitalOutTask(config.shutter_device,config.shutter_line_two,'Shutter2Ph')
    shutter2ph.off()
    power2ph = OnDemandVoltageOutTask(config.eom_device, config.eom_ao, 'Power2Ph')
    
    shutter3ph = OnDemandDigitalOutTask(config.shutter_device,config.shutter_line_three,'Shutter3Ph')
    shutter3ph.off()    
    #    galvos_controller.set3PhPowerMotor(power_motor)

#    galvos_controller.setLaser(laser)
    # Define where the galvos take their signal from
    galvos=Galvos(config.galvos_device,config.galvos_aox,config.galvos_aoy)
    galvos.config()
    
    print('Initialising microscope motors...')
    motors = Motorsclass(config.motorscomport, config.motorsbaud)
    print('...done!')
    
    print('Initialising 3P rotative motor...')
    rotmotor3P = ThorlabsMotor(config.rotMot3P_SN, config.rotMot3P_HW)
    print('...done!')

#   power_motor = RotationMotor(config.laser_intensity_device)
    #laser = Maitai(config.comPortLaser)    
    #replaser = laser.ReadStatus();
    #print 'status laser:' + replaser
    # Define scale of system (according to telescope and objective)
    # Thorlabs galvo mirrors provide 0.8 V per degree but this has to be calibrated on target
    radians_per_volt = 2*np.pi/(360.0)#1.06 corrected for small, 0.5 is for big
    f1=54
    f2=300
    fobj=7.2 #29.5 with 4x, 7.2 with 25x
    scale_um_per_volt=(2*fobj*f1/f2*radians_per_volt)*1000
    unit_converter = Converter()
    unit_converter.setScale(scale_um_per_volt,scale_um_per_volt)    
    galvos.setUnitConverter(unit_converter)
    
    # Set acquisition task that will be triggered on the galvo, try to build it
    # as a slave task
    ai_meas = AnalogInputTask(config.pmt_device,config.pmt,2)
    galvos.setSynchronizedAITask(ai_meas)
    
    # Set viewers, update at 10 Hz based on qt timer, can skip images if too slow
    viewer = ChannelViewer('Channel 0',0)
    po2viewer = po2Viewer('PO2 plot')
    viewer2 = ChannelViewer('Channel 1',500)
    ai_meas.setDataConsumer(viewer,False,0,'viewer',True)
    ai_meas.setDataConsumer(viewer2,False,1,'viewer',True)
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(viewer.update)
    timer.start(100)
    timer2 = pg.QtCore.QTimer()
    timer2.timeout.connect(viewer2.update)
    timer2.start(100)
    
    # Start the gui controlling the galvos
    galvos_controller = GalvosController()
    galvos_controller.setGalvos(galvos)
    galvos_controller.setShutter2Ph(shutter2ph)
    galvos_controller.setShutter3Ph(shutter3ph)
    galvos_controller.setAoEOM(power2ph)
    galvos_controller.setMotors(motors)
    #galvos_controller.setMaitai(laser)
    galvos_controller.setThorlabs(rotmotor3P)
    galvos_controller.setAiTask(ai_meas)
    galvos_controller.set_brain_pos_reset()
    galvos_controller.setImageViewer(viewer)
    galvos_controller.setImageViewer2(viewer2)
    galvos_controller.setPO2Viewer(po2viewer)

    galvos_controller.move(150,0)
    galvos_controller.resize(500,500)
    galvos_controller.show()

    

    #galvos.ao_task.signal_helper.aoDoneSignal.connect(galvos_controller.nextAcq)    
    
    #timer3 = pg.QtCore.QTimer()
    #timer3.timeout.connect(galvos_controller.checkPowerStatus_maitai)
    #timer3.start(600000) #checks power status every minute

    # Start the gui controlling the acquisition (save, views, etc.)
    app.exec_()  
    #timer3.stop()
    
    timer.stop()
    timer2.stop()    
    galvos_controller.turnOffWheel3P()
    galvos_controller.closeLaser()
    print 'killing GUI'