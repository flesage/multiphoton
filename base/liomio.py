# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 12:09:59 2015

@author: flesage
"""

"""
Created on Sat Sep 12 07:58:44 2015

@author: flesage
"""
import os
import posixpath
import numpy as np
import h5py
import nibabel as nb
from Queue import Empty
from multiprocessing import Process, Queue, Pipe
import math
import time
from main import config

class DataSaver():
    def __init__(self,filename):
        self.filename=filename
        print(self.filename)
        self.f = h5py.File(self.filename, 'a')
        print('h5py created')
        self.pathName = 'dummy'
        self.block_size = 512

    def setBlockSize(self,block_size):

        self.block_size = block_size
        if block_size < 512:
            block_size = 512
        self.queue = Queue(2*block_size)

#    def setDatasetName(self,dataset_name):
#        print(self.datasetname)
#        self.datasetname = dataset_name
#        self.f.create_group(posixpath.join('/scans',self.datasetname))
#        print(self.datasetname)
        
    def setDatasetName(self,path_name):
        self.pathName=path_name
        self.f.create_group(self.pathName)
        print(self.pathName)

    def addAttribute(self,attribute,value):
        self.f[self.pathName].attrs[attribute]=value
        

    def startSaving(self):

        # Thread to display so that the puts in queue are very fast
        print('in start saving loop')
        self.f.close()
        self.parent_conn, child_conn = Pipe()
        print('launching process')
        self.p = Process(target=save_process,args=(self.queue,self.block_size,self.pathName,child_conn,self.filename))
        print('start!')
        self.p.start()
        return self.parent_conn

    def stopSaving(self):
        # Stop saving, join save thread and clear data in queue to prepare
        # for next round
        print('parent sends False to child...')        
        self.parent_conn.send([False])      
        print('joining...')        
        self.p.join()

    def put(self,value,flag):
        self.queue.put(value,flag)
        
    def checkAlreadyExistingFiles(self,path_name,scanType):
        e = True
        counter = 0;
        while e:
            counter=counter+1
            self.pathName=posixpath.join(path_name,scanType+'_'+str(counter))
            print self.pathName
            e = self.pathName in self.f
            print e
            print counter
        return counter

def save_process(queue,block_size,datasetname,conn,filename):

    print(filename)
    f = h5py.File(filename, 'a')
    save_index = 0
    index = 0
    started = True
    print('in loop')
    while started:
        try:
            # We block for 2 seconds on no data to optimize CPU and avoid
            # running this while loop for no reason
            data = queue.get(True,1)
            # Create data buffer
            print(data.ndim)
            
            if data.ndim>1:
                if index == 0:
                    buf=np.zeros((data.shape[0],data.shape[1],block_size),dtype='int16')
                buf[:,:,index]=data
            else:
                if index == 0:
                    buf=np.zeros((data.shape[0],block_size),dtype='int16')
                buf[:,index]=data
            index = (index + 1) % block_size
            print(index)
            if index == 0:
                f[posixpath.join(datasetname,'scan'+u'%05d' % save_index)]=buf
                save_index = save_index + 1
                conn.send([save_index,queue.qsize()])
        except Empty:
            print('no data')
            if conn.poll():
                print('checking connection status')
                started = conn.recv()[0]
                print(started)
            # Reloop
            pass
    # Save last acquisition if incomplete (duplicate if exactly complete?)
    if data.ndim>1:
        buf2=np.zeros((data.shape[0],data.shape[1],index+1),dtype='int16')
        buf2=buf[:,:,0:index];
    else:
        buf2=np.zeros((data.shape[0],index+1),dtype='int16')
        buf2=buf[:,0:index];        
    f[posixpath.join('/scans',datasetname,'scan'+u'%05d' % save_index)]=buf2
    f.close()


class NiiStackSaver():
    def __init__(self,pathname,nx,ny,nz,n_extra,topposition,zstep,saving,imageprecheck,current_slice,i,j,channel):
        self.channel = channel
        self.z_index = 0
        self.nx = nx
        self.ny = ny
        self.i = i
        self.j = j
        self.saving = saving
        self.n_extra = n_extra
        self.top = topposition
        self.zstep = zstep
        self.nz = nz
        self.current_slice=current_slice
        self.pathname=pathname
        self.image = np.memmap('C:\\Users\\LiomW17\\Desktop\\test\\test' + str(self.channel)+ '.dat',dtype='uint16',mode='r+',shape=(ny,nx+n_extra,nz))
        self.stepinx = 0
        self.stepiny = 0
        self.affine = np.eye(4)
        self.Precheck = False
        self.contrasteimage = 0
        self.imageprecheck = imageprecheck
        self.exception = False


        
    def put(self,data,flag):
        
        #print str(self) + ' z index: ' + str(self.z_index) + ' mean data: ' + str(np.mean(np.ravel(data)))
        
        self.image[:,:,self.z_index] = data + 32768
        
        self.z_index = self.z_index+1
        
        
        
        
        
        #Détection d'organe, faire une matrice et considérer les images intéressantes
        
        if self.z_index == self.nz and self.saving == False:
        
            
            self.imageminz = np.amin(self.image[:,:,:], axis = 2) # min en z
          
            ##self.std = np.std(self.imageminz)
            
            self.P = np.ravel(self.imageminz)
            
            
            
            self.contrasteimage = 0
            
            for data in self.P :
                if data < config.tresholdtriggerimage :
                    self.contrasteimage = self.contrasteimage +1
                    #print self.contrasteimage
            
            print self.contrasteimage,'/',len(self.P),'=', 100.0*float(self.contrasteimage)/float(len(self.P)), '%'
            ##print 'Std = ', self.std, ', threshold = ', config.tresholdtriggerimage
            
            if self.contrasteimage > 0.10*float(len(self.P))  : #10%
             
                ##if self.std > config.tresholdtriggerimage:
                self.imageprecheck[self.j,self.i,0]= 1.
                print('tissus détecté')
                #print self.imageprecheck
            else:
                self.imageprecheck[self.j,self.i,0]= 2.
                
                print ('tissus non détecté')
                
        
            #Détection de surface_ utilisation du gradient
                
      
            if self.imageprecheck[self.j,self.i,0] == 1:
                
                self.imgmeanxy = np.mean(self.image[:,:,:], axis = (0, 1)) # mean en x et y
                
                self.etage = np.zeros((1,len(np.ravel(self.image[:,:,0]))))
                
                self.detection = np.zeros((1,len(np.ravel(self.image[0,0,:]))))
                
                for i in range(len(self.image[0,0,:])) :
                    self.etage = np.ravel(self.image[:,:,i])
                    #print i , 'i detectionsurface'
                    
                    
                    self.contrastesurface = self.etage < config.tresholdtriggerimage                  
                    
                    self.detection[0,i] = len( self.etage[self.contrastesurface] )
                    
                    #for j in range(len(self.etage)):
                         
                        #if self.etage[j] < config.tresholdtriggerimage :
                            #self.contrastesurface = self.contrastesurface +1
                    
                    #self.detection[0,i] =  len(self.contrastesurface)
                
                print self.detection
                self.surfacevalue = float(np.argmax(self.detection)) - 3.0  # ajouter plus de mesures dans le gui
                    
                #print self.imgmeanxy
                ##self.imggradxy = -np.gradient(self.imgmeanxy)
                #print self.imggradxy , 'self.imggradxy'
                ##self.surfacevalue = None
                
                ##for i in range(len(self.imgmeanxy)):
                        
                    ##if  self.imggradxy[i] > config.tresholdtriggersurface:
                        ##self.surfacevalue = i
                        ##break
                    ##if self.surfacevalue is None and i == len(self.imgmeanxy) - 1:
                        ##self.surfacevalue = float(np.argmax(self.imggradxy))

                #print self.surfacevalue , 'self.surfacevalue'
                self.imageprecheck[self.j,self.i,1]= self.top - (self.surfacevalue * self.zstep/1000.0) # a cause de stack_thread_precheck dans GalvosController
                #print self.imageprecheck


        if self.z_index == self.nz:
            self.z_index = 0


    def save(self,i,j,current_slice,channel):
        
        self.i = i
        self.j = j
        self.current_slice = current_slice
        self.channel = channel
        
        #print 'CH :'+ str(self.channel)+ ' data:' +str(np.mean(np.ravel(self.image)))
  
        self.img = nb.Nifti1Image(self.image[:,:,:],self.affine)
        self.filocation= os.path.join(self.pathname, 'channel_'+str(self.channel)+'_nx_'+ str(self.j) + '_ny_'+ str(self.i) + '_slice_'+ str(self.current_slice) + '.nii.gz')
        nb.save(self.img, self.filocation)

    
          
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
