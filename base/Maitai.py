# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 11:09:26 2015

@author: PMartel
"""

import serial

class Maitai(): 
    def __init__(self, comPort):
        self.serial = serial.Serial()
        self.serial.baudrate = 9600#115200#
        self.serial.port = comPort
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.timeout = 3
    
    def SetWatchdogto0(self):
        command = 'TIM:WATC 0\r'
        self.write_command(command)
    
    def ReadStatus(self):
        command = '*STB?\r'
        response = self.write_command(command,10)
        return response
    
    def OpenLaser(self):
        
        self.SetWatchdogto0()
        
        wavelength=850
        print 'setting wavelength'
        command= 'WAV '+str(wavelength)+'\r'
        response = self.write_command(command,10)        
        while (response[0:-3]!='850'):
            command= 'WAV?\r'
            response = self.write_command(command,10)
        print 'wavelength set!'
        command = '*STB?\r'
        response = self.write_command(command,10)
        print 'STB = ',response
        command= 'READ:PCTW?\r'
        response =self.write_command(command,3)
        print 'Warm up = ',response,'%'
        while response!='100':
            command= 'READ:PCTW?\r'
            response = self.write_command(command,3)
            print 'Warm up = ', response, '%'
        print 'turning laser: ON'
        command = 'ON\r'
        self.write_command(command)
        print '...done!'     
        while int(response)!=15: 
            command = '*STB?\r'
            response =self.write_command(command,10)
            command = 'READ:PLAS:PCUR?\r'
            current = self.write_command(command,1000)
            print 'Opening:', response, '- Diode current:',current
        print 'Laser Opening terminated'
        
    def CloseLaser(self):
        command = 'OFF\r'
        self.write_command(command)
        print('Laser is now off')
        
        
    def ShutDown(self):
        print'does not seem to be available in MaiTai lasers ...'
        #command = 'SHUTDOWN\r'
        #self.write_command(command)
        
    def WarmedUp(self):
        command= 'READ:PCTW?\r'
        self.response = self.write_command(command,3)
        return self.response
        
        
    def Identify(self):
        command= '*IDN?\r'
        self.write_command(command, 1024)
    
        
    def SHUTTERshut(self):
        command= 'SHUT 0\r'
        self.write_command(command)    
        
    def SHUTTERopen(self):
        command= 'SHUT 1\r'
        self.write_command(command)
        
    def ChangeWavelength(self, wavelength):
        
        if isinstance(wavelength, basestring) is True:
            command= 'WAV '+wavelength+'\r'
        else:
            command= 'WAV '+str(wavelength)+'\r'
        wavelength_set=self.write_command(command,1000)
        return wavelength_set
    
    def CheckPower(self):
        command= 'READ:POW?\r'
        self.write_command(command)
    
    def write_command(self,command,responseLength=None):
        
        self.serial.open()
        self.serial.flushInput()
        self.serial.flushOutput()
        self.serial.write(command)

        # Read the command response (if any)
        if responseLength is None:
            response = self.serial.readline()
        elif responseLength > 0:
            response = self.serial.read(responseLength)
        else:
            response = None
        self.serial.flushInput()
        self.serial.flushOutput()  
        self.serial.close()

        return response
    
    def ReadHumidity(self):
        command = 'READ:HUM?\r'
        hum = self.write_command(command,10)
        print hum
    
    def ReadPower(self):
        command = 'READ:POW?\r'
        power = self.write_command(command,10)
        return power
    
    def ReadCurrent(self):
        command = 'READ:PLAS:PCUR?\r'
        current = self.write_command(command,1000)
        return current
    
    def ReadWavelength(self):
        command = 'WAV?\r'
        wavelength = self.write_command(command,10)
        return wavelength
    
    def ReadMotor(self):
        print'does not seem to be available in MaiTai lasers ...'
#         command = 'CONT:DSMPOS?\r'
#         motorpos = self.write_command(command,10)
#         return motorpos
    
    
    def movemotor(self,pos):
        print'does not seem to be available in MaiTai lasers ...'
#         command = 'CONT:DSMPOS ' +str(pos)+ '\r'
#         motorpos = self.write_command(command,10)
#         return motorpos
    
    def read_maxandmin_motor(self):
        print'does not seem to be available in MaiTai lasers ...'
#         command = 'CONT:SLMIN?\r'
#         motormin = self.write_command(command,10)
#         command = 'CONT:SLMAX?\r'
#         motormax = self.write_command(command,10)
#         return motormin,motormax
    
    def __del__(self):
        self.CloseLaser()
        self.serial.close()
    
    
    def read_pumplasererrorcode(self):
        command = 'PLAS:ERRC?\r'
        plaserr = self.write_command(command,10)
        return plaserr

    

