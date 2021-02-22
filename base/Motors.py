# -*- coding: utf-8 -*-


import serial
import time
import binary
import APTMotor
import Queue

from serial import SerialException
'''
import math
'''
#This script is used to control the 2photon microscope Z axis motor
class piezoClass():
    def __init__(self,comPort,baud):
        print('open piezo')
        self.comPort = comPort
        self.baud=baud
        
        self.ser=serial.Serial(port=self.comPort, baudrate=self.baud, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=1,writeTimeout=1)
        if self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()              
            self.ser.close()
        self.ser.open()   
        self.ser.flushInput()
        self.ser.flushOutput()   
        self.ser.write('1MO\r')
        self.ser.flushInput()
        self.ser.flushOutput()      
        self.piezoMoving=0
        print('port open!')     
        
    def closeMotor(self):
        if self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()   
            self.ser.write('1MO\r')
            self.ser.flushInput()
            self.ser.flushOutput()                       
            self.ser.close() 
            print('port closed!')       
        else:
            print('port already closed!')
            
    def startJog(self,val):
        if self.ser.is_open:
            command='1JA'+str(val)+'\r'
            print(command)
            self.ser.flushInput()
            self.ser.flushOutput()   
            self.ser.write(command)
            self.ser.flushInput()
            self.ser.flushOutput()
            time.sleep(0.1)
            self.ser.write(command)
            self.ser.flushInput()
            self.ser.flushOutput()                        
            print('jog started')   
            self.piezoMoving=1   
        else:
            print('port closed!')   
            
    def stopMotor(self):
        if self.ser.is_open:
            if self.piezoMoving:
                self.ser.flushInput()
                self.ser.flushOutput()   
                self.ser.write('1JA0\r')
                self.ser.flushInput()
                self.ser.flushOutput() 
                time.sleep(0.1) 
                self.ser.write('1ST\r')
                self.ser.flushInput()
                self.ser.flushOutput()                         
                print('motor stopped')  
                self.piezoMoving=0     
            else:
                print('motor already stopped')
        else:
            print('port closed!')    
            
    def goToPosition(self,distance):    
        self.nm_per_ustep=10
        distance_in_nm=distance*1000*1000
        distance_in_usteps=distance_in_nm/self.nm_per_ustep
        if self.ser.is_open:
            command='1PR'+str(distance_in_usteps)+'\r'
            self.ser.flushInput()
            self.ser.flushOutput()   
            self.ser.write(command)
            self.ser.flushInput()
            self.ser.flushOutput()                       
            print('movement started')   
            self.piezoMoving=1   
        else:
            print('port closed!')
            
    def setStopFlag(self,flag):
        self.piezoMoving=flag
        
    def setSpeed(self,val):
        self.speed=val
        
    def piezoOscillateUpdate(self):
        self.speed=-self.speed
        self.startJog(self.speed)
            
    def piezoOscillate(self,numIterations,timePerIteration,speed):
        if self.ser.is_open:
            for i in range(numIterations):
                if self.piezoMoving:
                    print('iteration:'+str(i)+'out of'+str(numIterations))
                    command='1JA'+str(speed)+'\r'
                    print(command)
                    self.ser.flushInput()
                    self.ser.flushOutput()   
                    self.ser.write(command)
                    self.ser.flushInput()
                    self.ser.flushOutput()
                    time.sleep(0.1)
                    self.ser.write(command)
                    self.ser.flushInput()
                    self.ser.flushOutput()                        
                    print('jog started')     
                    time.sleep(timePerIteration)
                    speed=-speed
                else:
                    self.stopMotor()
            self.stopMotor()
        else:
            print('port closed!')      


class Motorsclass:
    def __init__(self,comPort,baud):
        self.comPort = comPort
        self.baud=baud
        
        self.ser=serial.Serial(port=self.comPort, baudrate=self.baud, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=1,writeTimeout=1)
        if self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()              
            self.ser.close()
        self.ser.open()   
        self.ser.flushInput()
        self.ser.flushOutput()   
        self.ser.write('1MO\r')
        self.ser.write('2MO\r')
        self.ser.write('3MO\r')
        #self.home()
    
    def home(self):
#        Wait until homing is done to return from this function to
#        make sure we have accurate positioning
        #Motor 1
        print('Homing X Motor')
        done=0
        self.ser.write('1OR0\r')
        #self.ser.write('1PA0\r')
        while not done:                
                self.ser.write('1MD?\r')
                done = self.ser.read(4);
                time.sleep(1)
                
        #self.ser.write('1PA0\r')
        #Motor 2
        #=======================================================================
        print('Homing Y Motor')
        done=0
        self.ser.write('2OR0\r')
        while not done:
                self.ser.write('2MD?\r')
                done = self.ser.read(4);
                time.sleep(1)
        #=======================================================================
        #Motor 3
        #=======================================================================
        print('Homing Z Motor')
        done=0
        self.ser.write('3OR0\r')
        while not done:
                self.ser.write('3MD?\r')
                done = self.ser.read(4);
                time.sleep(1)
        #=======================================================================

#        Center each motor in the center of their stride
                
        #Motor 1
        #self.move_dx(-27)
        #self.ser.write('1PA12.5\r')
        #while not done:
        #        self.ser.write('1MD?\r')
        #        done = self.ser.read(4);
        #        time.sleep(1)
#        #Motor 2
#        print('Homing Y Motor')
#        done=0
#        self.ser.write('1PA12.5')
#        while not done:
#                self.ser.write('2MD?')
#                done = self.ser.fscanf('%d');
#                time.sleep(1)
#        #Motor 3
#        print('Homing Z Motor')
#        done=0
#        self.ser.write('3PA0.95')
#        while not done:
#                self.ser.write('3MD?')
#                done = self.ser.fscanf('%d');
#                time.sleep(1)
        
    def move_dx(self,dx):
        #print(dx)
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write('1PR'+str(dx)+'\r')
        data=0
        while not data:
            time.sleep(0.1)
            self.ser.write('1MD?\r')
            data = self.ser.read(1);
            
    def get_velocity_x(self):
        self.ser.flushInput()
        self.ser.flushOutput()        
        print 'get_pos: in function'
        data=0
        while not data:
            self.ser.write('1VA?\r')
            data = self.ser.read(1000)
        time.sleep(0.1)  
        print 'get velocity: done'
        return data     
    
    def set_velocity_x(self,velocity_x):
        self.ser.flushInput()
        self.ser.flushOutput()        
        data=0
        self.ser.write('1VU'+str(0.4)+'\r')
        self.ser.write('1VB'+str(velocity_x)+'\r')

        self.ser.write('1VA'+str(velocity_x)+'\r')
        while not data:
            self.ser.write('1MD?\r')
            data = self.ser.read(1);
            time.sleep(0.1)    
 
    def move_dy(self,dy):
        self.ser.flushInput()
        self.ser.flushOutput()        
        data=0
        self.ser.write('2PR'+str(dy)+'\r')
        while not data:
            self.ser.write('2MD?\r')
            data = self.ser.read(1);
            time.sleep(0.1)        

    def move_dz(self,dz):
        self.ser.flushInput()
        self.ser.flushOutput()        
        done=0
        self.ser.write('3PR'+str(dz)+'\r')
        while not done:
            self.ser.write('3MD?\r')
            done = self.ser.read(1);
            time.sleep(0.1)    
          
    def move_ax(self,x_pos):
        self.ser.flushInput()
        self.ser.flushOutput()        
        data=0
        self.ser.write('1PA'+str(x_pos)+'\r')
        while not data:
            self.ser.write('1MD?\r')
            data = self.ser.read();
            time.sleep(0.1)
            
    def move_ay(self,y_pos):
        self.ser.flushInput()
        self.ser.flushOutput()        
        done=0
        self.ser.write('2PA'+str(y_pos)+'\r')
        while not done:
            self.ser.write('2MD?\r')
            done = self.ser.read(1000);
            time.sleep(0.1)        
            
    def move_az(self,z_pos):
        self.ser.flushInput()
        self.ser.flushOutput()           
        self.ser.write('3PA'+str(z_pos)+'\r')
        done=0
        while not done:
            self.ser.write('3MD?\r')
            done = self.ser.read(1000);
            time.sleep(0.1)  
                
    def get_pos_axial(self):
        self.ser.flushInput()
        self.ser.flushOutput()        
        print 'get_pos: in function'
        data=0
        while not data:
            self.ser.write('3TP\r')
            data = self.ser.read(1000)
        time.sleep(0.1)  
        print 'get_pos: done'
        return data
    
    def get_pos(self,axisVal):
        self.ser.flushInput()
        self.ser.flushOutput()        
        print 'get_pos: in function'
        data=0
        while not data:
            self.ser.write(str(axisVal)+'TP\r')
            data = self.ser.read(1000)
        time.sleep(0.1)  
        print 'get_pos: done'
        return data
    
            #self.ser.write('2TP')
#             y = self.ser.read()
#             self.ser.write('3TP')
#             z = self.ser.read()
    def getouttheway(self):
        self.ser.write('1PA' + str(13)+'\r')
        
    def delete(self):
        self.ser.write('1MF\r')
#             self.ser.write('2MF')
#             self.ser.write('3MF')
            
        self.ser.close()
        self.ser.delete()
            
            
""" This class contains all the linear Zaber connected in series. If there are
 more or less motors than presently (x and y), it is simply possible to 
 add them to the class attributes as a new BinaryDevice instance. 
 https://www.zaber.com/products/product_detail.php?detail=T-LSR150B """
class ZaberMotor():
    def __init__(self,comport):

        try:
            self.ser = binary.BinarySerial(comport)
            
            self.motorx = binary.BinaryDevice(self.ser, 1)
            self.motory = binary.BinaryDevice(self.ser, 2)
            self.motorCARS = binary.BinaryDevice(self.ser, 3)
            self.motorCARS2 = binary.BinaryDevice(self.ser, 4)
            self.microstepsize=0.49609375
            self.Config=False
            
        except SerialException:
            print('Port already open, motor instances not created')
        
            
    def setConfig(self,maxzaberx,maxzabery,maxzaberxpipe,minzaberypipe):
        self.Config=True
        self.maxzaberx= maxzaberx
        self.maxzabery= maxzabery
        self.maxzaberxpipe= maxzaberxpipe
        self.minzaberypipe= minzaberypipe
        
    def getPosmainline(self):
        poss = self.motorCARS.get_pos()
        return poss   
            
    def getPos(self):
        poss = [0, 0]
        poss[0] = self.motorx.get_pos()  #in number of microsteps
        poss[1] = self.motory.get_pos()
        return poss
    
    def goHome(self):
        print('Moving motors home...')
        self.motorx.home()
        self.motory.home()
        
        
    def goTo(self, pos):
        
        #Only if you have configurations on your Zabers (may need modifications)
        if self.Config :
            self.poss = [0, 0]
            
            self.getposs()
            
            self.expectx= self.poss[0] + (pos[0]*1000/self.microstepsize)
            self.expecty= self.poss[1] + (pos[1]*1000/self.microstepsize)
            
            self.differencemouvx=0
            self.differencemouvy=0
            #print('Valeur expecte en x',self.expectx)
        
            if (self.expecty < self.minzaberypipe) and (self.expectx > self.maxzaberxpipe) :
                self.differencemouvx= (pos[0]*1000/self.microstepsize)-self.maxzaberxpipe
                self.posmaxxpipe=self.maxzaberxpipe-self.poss[0]
                self.motorx.move_rel(self.posmaxxpipe)
                print('Attention au poteau(zaber x)')
        
            elif self.expectx < self.maxzaberx :
                self.motorx.move_rel(pos[0]*1000/self.microstepsize)   #enter displacement in mm
                
            elif self.expectx > self.maxzaberx :
                self.posmaxx=self.maxzaberx-self.poss[0]
                self.motorx.move_rel(self.posmaxx)
                print('max x')     
                
            self.getposs()
            self.expecty= self.poss[1] + (pos[1]*1000/self.microstepsize)
            
        
            if (self.expecty < self.minzaberypipe) and (self.expectx > self.maxzaberxpipe) :
                self.differencemouvy= (pos[1]*1000/self.microstepsize)-self.minzaberypipe
                self.posminypipe=self.minzaberypipe-self.poss[1]
                self.motory.move_rel(self.posminypipe)
                print('Attention au poteau(zaber y)')
                
            elif (self.expecty < self.maxzabery):
                self.motory.move_rel(pos[1]*1000/self.microstepsize)
                
            elif self.expecty > self.maxzabery :
                self.posmaxy=self.maxzabery-self.poss[1]
                self.motory.move_rel(self.posmaxy)
                print('max y')
                
            self.getposs()
                
            #if (self.differencemouvx+ self.poss[0] ) < self.maxzaberx :
                #self.motorx.move_rel(self.differencemouvx)
                
            #elif (self.differencemouvx+ self.poss[0] )> self.maxzaberx :
                #self.motorx.move_rel(self.maxzaberx - self.poss[0])
                #print('max x')
                     
            #self.getposs()      
            
                
                
        else:
            self.motorx.move_rel(pos[0]*1000/self.microstepsize)
            self.motory.move_rel(pos[1]*1000/self.microstepsize)
            
        
               
            
    def goTo_abs(self, pos):
        self.motorx.move_abs(pos[0]*1000/self.microstepsize)   #enter displacement in mm
        self.motory.move_abs(pos[1]*1000/self.microstepsize)
        
        
    def move_x(self,position):
        
        if self.Config :
            
            self.getposs()
            #print(self.poss[0],self.poss[1])
            self.expecty= self.possy 
            self.expectx= (position*1000/self.microstepsize)
            
            #print('Valeur expecte en x',self.expectx)
        
            if (self.expecty < self.minzaberypipe) and (self.expectx > self.maxzaberxpipe) :
                #self.differencemouvx= (pos[0]*1000/self.microstepsize)-self.maxzaberxpipe
                #self.posmaxxpipe=self.maxzaberxpipe-self.poss[0]
                self.motorx.move_abs(self.maxzaberxpipe)
                print('Attention au poteau(zaber x)')
        
            elif self.expectx < self.maxzaberx :
                self.motorx.move_abs(position*1000/self.microstepsize)   #enter displacement in mm
                
            elif self.expectx > self.maxzaberx :
                #self.posmaxx=self.maxzaberx-self.poss[0]
                print('max x')     
                self.motorx.move_abs(self.maxzaberx)
                
            self.getposs()
            #print(self.poss[0],self.poss[1])
                
                
        else:
            self.motorx.move_abs(position*1000/self.microstepsize)      #enter displacement in mm
        
        
    def move_y(self,position):
        
        if self.Config :
            
            self.getposs()
            #print(self.poss[0],self.poss[1])
            
            self.expecty= (position*1000/self.microstepsize)
            self.expectx= self.possx
            
            if (self.expecty < self.minzaberypipe) and (self.expectx > self.maxzaberxpipe) :
                #self.differencemouvy= (pos[1]*1000/self.microstepsize)-self.minzaberypipe
                #self.posminypipe=self.minzaberypipe-self.poss[1]
                self.motory.move_abs(self.minzaberypipe)
                print('Attention au poteau(zaber y)')
                
            elif (self.expecty < self.maxzabery):
                self.motory.move_abs(position*1000/self.microstepsize)
                
            elif self.expecty > self.maxzabery :
                #self.posmaxy=self.maxzabery-self.poss[1]
                self.motory.move_abs(self.maxzabery)
                print('max y')
                
            self.getposs()
            #print(self.poss[0],self.poss[1])
        else:
            self.motory.move_abs(position*1000/self.microstepsize)      #enter displacement in mm
        
    def getposs(self):
            self.poss= [0,0]
            self.poss[0] = self.motorx.get_pos()
            self.possx= self.poss[0]
            self.poss[1] = self.motory.get_pos()
            self.possy= self.poss[1]
            
        
        
class RotationMotor():   
    def __init__(self,comport):
        try:
            self.ser = binary.BinarySerial(comport, 9600, 5)        
            motorAll = binary.BinaryDevice(self.ser, 0) #0 is for all motors
            self.motor = [None]
            nDevices = motorAll.renumberAndCount()
            if nDevices >= 1:
                self.motor = [None] * (nDevices + 1)
                self.motor[0] = motorAll;
                for dev in range(1, 1+nDevices):
                    self.motor[dev] = binary.BinaryDevice(self.ser, dev)
            print "Found " + str(nDevices) + " rotation motors."
                
            self.micro_step_size = 0.000234375;
            #self.d=[0,4.5 ,6.3, 7.2, 9, 18, 27, 36, 45, 49.5, 51.3, 54,63, 72, 81, 90]
            #self.pwr=[0.810,0.1825,0.120,0.150,0.285,2.85,7.6,12.85,16.45,18.6,18.6,17,14.4,9.6,4.35,0.720]
            #self.npwr=self.pwr/max(self.pwr)
            #self.y=0.5*math.sin((math.pi*self.d/180)*4-(4*math.pi/8)-(4*15*math.pi/180))+0.5
            #self.deg=45/math.pi*(math.asin(2*percent-1)+5*math.pi/6)
            try:
                #print "Rotation motor renumber return: " + comport + str(r)
                self.goHome()
            except:
                print 'Problem homing rotation motor'
                pass
        except SerialException:
            print('Port already open, motor instances not created')
    def getPos(self, dev):
        pos = self.motor[dev].get_pos()*self.micro_step_size 
        return pos
            
    def goHome(self, dev = 0):
        try:
            self.motor[dev].home()
        except:
            pass
        
    def goTo(self, dev, angle_deg):
        try:
            self.motor[dev].move_rel(angle_deg/self.micro_step_size)
        except:
            pass
    
    def goTo_abs(self, dev, angle_deg):
        try:
            self.motor[dev].move_abs(angle_deg/self.micro_step_size)
        except Exception,e: 
            print str(e)
            pass
    
        
""" This class holds the code to initialize and use a ThorLabs LabJack motor.
 Plenty more methods of moving the motor and getting information are available
 when using the APTMotor instance directly.  """
class ThorlabsMotor(APTMotor.APTMotor):
    def __init__(self, SN, HWTYPE):
        self.configthorlabsmotor=False
        super(ThorlabsMotor, self).__init__(SN, HWTYPE)
        self.motor =  APTMotor.APTMotor(SN, HWTYPE) # SN and motor type
        self.motor.initializeHardwareDevice()
        self.startPosition=0
        #self.verbose = " "
        
#        self.initializeHardwareDevice()
    def setLimits(self,maxthorlabsmotor):
        self.configthorlabsmotor=True
        self.maxtlabsmotor= maxthorlabsmotor
        
    def goHome(self):
        self.configthorlabsmotor=False
        self.mAbs(0.0)
        self.configthorlabsmotor=True
        
    def setpos(self,commandPos):
        self.configthorlabsmotor=False
        print 'going to position...'
        self.mAbs(commandPos)
        print 'done!'
        self.configthorlabsmotor=True
        
    def setposRel(self,commandPos):
        self.configthorlabsmotor=False
        print 'moving of ' + str(commandPos) + '...'
        self.mRel(commandPos)
        print 'done!'
        self.configthorlabsmotor=True
        
    def setStartPosition(self):
        self.startPosition=self.motor.getPos() 

    def getpos(self):
        currentPosition=self.motor.getPos() 
        return currentPosition
        
    def clean_up_APT(self):
        self.cleanUpAPT()
        
    def __del__(self):
        self.cleanUpAPT()




class LabjackComport():
    def __init__(self,comPort):
        self.transitTime = 500.0; # ms
        self.transitTime_ADC = self.convert_ms2ADC(self.transitTime);
        self.channel = 0x00

        self.destination = 0x50 # General USB Port
        self.source = 0x01 # Host
        self.serial = serial.Serial()
        self.serial.baudrate = 115200
        self.serial.port = comPort
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.timeout = 10
        
        self.sethomeparam()
    
    def identify(self):
        """Instruct hardware unit to identify itself (by flashing its front panel LEDs) """
        command = [0x23, 0x02, 0x00, 0x00, self.destination, self.source]
        output = self.write_command(command);
        print output
        
    
    def goHome(self):
        
        command = [0x44, 0x04, self.channel, 0x00, self.destination, self.source]
        self.write_command(command)

    def write_command(self,command, convertHex=True, responseLength=None):
        assert isinstance(command, list), "'command' must be a list"

        if convertHex:
            for ii, x in enumerate(command):
                if isinstance(x, str):
                    command[ii] = int(x,16)
                else:
                    command[ii] = int(x)
        
        print command
        # Write a command
        self.serial.close()
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
        self.serial.close()

        return response
    
    def sethomeparam(self):
        command = [0x40, 0x04, 0x0e, 0x00, self.destination, self.source, 0x01, 0x00,0x02,0x00,0x01,0x00,0xF4,0x70,0xEE,0x03,0x00,0xC0,0x03,0x00]
        self.write_command(command)
    
    def setmoveAbs(self):
        print ''
    
    def moveAbs(self,milimeters):
        string = hex(milimeters*409600)
        if len(string)>8:
            raise Exception('Thorlabs move error')
        str1 = string[2:]
        fill = 8 - len(str1)
        filler = '0'*fill
        str2 = filler + str1
        add = '0x'
        command = [0x53,0x04,0x00,0x00,self.destination,self.source,0x01,0x00,add+str2[6:],add+str2[4:6],add+str2[2:4],add+str2[0:2]]
        print command
        response = self.write_command(command)
        
        print int(response.encode('hex'),16)
        
        
    def getInfo(self):
        command = [0x05,0x00,0x00,0x00,self.destination,self.source]
        response = self.write_command(command,responseLength=400)
        print response
        
    def setmax(self,maximum):
        print ('max_to_code: ',maximum)
    
    def convert_ms2ADC(self, time_ms):
        time_adc = int(1e7 * time_ms ** (-1.591))
        return time_adc