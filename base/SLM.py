# -*- coding: utf-8 -*-

''' Class to control the SLM for wavefront shaping
author: PMartel
inspired by : https://github.com/wavefrontshaping/slmPy
'''
import numpy as np
import math
from base import slmpy
import time



# Call slm via slmpy fisrt and give it to class :: slm = slmpy.SLMdisplay(monitor = 1)

def compute_slopes(xshift,yshift,fslm,flens1,flens2,fobj,d1,d2,d3,WD):
        
        #fslm not used for central approximation
        
        eps= d1 + d2 +(d1*d2/flens1)
        omeg= 1.0-(d1/flens1)
        gamma=(-1.0*eps/flens2)+omeg
        
        
        slopex = xshift/((eps+d3*gamma)+(WD*(-1.0/fobj)*(eps+d3*gamma) + gamma))
        slopey = yshift/((eps+d3*gamma)+(WD*(-1.0/fobj)*(eps+d3*gamma) + gamma))
        
        return slopex, slopey

def GS_algo(source,img,max_iter= 1000):
        # image is a np.meshgrid
        
        t1 = time.time()
        h, w = img.shape
        h1,w1 = source.shape
        if h1!= h or w1 != w :
            raise ValueError('Source and target image not same size')
        pm_s = np.random.rand(h, w)
        pm_f = np.ones((h, w))
        am_s = np.sqrt(img)
        am_f = source

        signal_s = am_s*np.exp(pm_s * 1j)

        for iter in range(max_iter):
            print iter
            signal_f = np.fft.fft2(signal_s)
            pm_f = np.angle(signal_f)
            signal_f = am_f*np.exp(pm_f * 1j)
            signal_s = np.fft.ifft2(signal_f)
            pm_s = np.angle(signal_s)
            signal_s = am_s*np.exp(pm_s * 1j)
        pm =pm_f
        #transform to 255 range
        
        #psf = (pm + np.pi)*255.0/(2*np.pi)
        tf=  time.time() -t1
        print tf
        #Output as int for the SLM
        return pm #psf.astype('uint8') #
        


class SLM_class():
    def __init__(self,monit=1):
        
        self.slm = slmpy.SLMdisplay(monitor = monit)
        self.resX, self.resY = self.slm.getSize()
        self.X,self.Y = np.meshgrid(np.linspace(-1.0*self.resX/2,self.resX/2,self.resX),np.linspace(-1.0*self.resY/2,self.resY/2,self.resY))
        self.phasemaplens = self.generate_fresnel_lens(f = 3.0 ,xshift=(15*10**(-2)), yshift=(13*10**(-2)), result_type = 'float')
        
        
    def GS_algo(self,source,img,max_iter= 1000,result_type='float'):
        # image is a np.meshgrid
        
        t1 = time.time()
        h, w = img.shape
        h1,w1 = source.shape
        if h1!= h or w1 != w :
            raise ValueError('Source and target image not same size')
        pm_s = np.random.rand(h, w)
        pm_f = np.ones((h, w))
        am_s = np.sqrt(img)
        am_f = source

        signal_s = am_s*np.exp(pm_s * 1j)

        for iter in range(max_iter):
            print iter
            signal_f = np.fft.fft2(signal_s)
            pm_f = np.angle(signal_f)
            signal_f = am_f*np.exp(pm_f * 1j)
            signal_s = np.fft.ifft2(signal_f)
            pm_s = np.angle(signal_s)
            signal_s = am_s*np.exp(pm_s * 1j)
        pm =pm_f
        #transform to 255 range
        
        #psf = (pm + np.pi)*255.0/(2*np.pi)
        tf=  time.time() -t1
        print tf
        recovery = np.fft.ifft2(np.exp(pm * 1j))
        
        np.save('C:\\Users\\LiomW17\\Desktop\\GS_algo_output',pm)
        np.save('C:\\Users\\LiomW17\\Desktop\\recovery',recovery)
        
        pm[pm<0]=pm[pm<0]+2*np.pi
        
        if result_type == 'uint8':
            pm = pm*255.0/(2*np.pi)
        
        #Output as int for the SLM
        return pm #psf.astype('uint8') #
        
        
    def display_image(self,img,addlens = False,addflip=True):
        
        
        
        
        if addlens:
            img1 = img + self.phasemaplens
            self.slm.updateArray(img1.astype('uint8'))
        elif addflip:
            img1 = np.flip(img,0)
            self.slm.updateArray(img1.astype('uint8'))
        else:
            self.slm.updateArray(img.astype('uint8'))
        
    def generate_fresnel_lens(self,L=800*(10**(-9)),f = 30*(10**(-2)),pixelsize =0.000008,xshift = 0,yshift=0, result_type ='slm',slopecomputed=False,slopex=0,slopey=0):
        
        self.L = L
        self.f = f
        self.a = 1.0/L #compute theta for given FOV and divide by L
        if slopecomputed ==True:
            self.a = (self.compute_slopes(xshift, fslm = 0, flens1= 0.4, flens2=0.254, fobj=0.07, d1=0.1, d2=0.654, d3=4, WD=0.01))/(xshift*L)
        self.xshift = xshift# in meters
        self.yshift = yshift# in meters
        ImgResX = self.resX
        ImgResY = self.resY
        X,Y = np.meshgrid(np.linspace(-1.0*ImgResX,ImgResX,ImgResX),np.linspace(-1*ImgResY,ImgResY,ImgResY))
        philens = (1/(2*f*L))*(pixelsize*pixelsize*X*X + pixelsize*pixelsize*Y*Y)
        phitranslation = self.a * (pixelsize*X*self.xshift + pixelsize*Y*self.yshift)
        phitot = (philens+phitranslation)%(2.0*np.pi)
        
        self.phasemap = phitot
        
        if result_type == 'slm':
            phitot1 = (phitot*255.0/(2.0*np.pi)).astype('uint8')
            return phitot1
        
        elif result_type == 'float':
            phitot1 = (phitot*255.0/(2.0*np.pi)).astype('float')
            return phitot1
         
        else :
            return phitot
        
    
    def generate_stack_move(self,img,xmin,xmax,ymin,ymax,xinc,yinc,result_type='float'):
        
        xmove = np.arange(xmin,xmax,xinc)
        ymove = np.arange(ymin,ymax,yinc)
        shift = self.generate_fresnel_lens(f = float('inf'))
        
        for x in xmove:
            for y in ymove:
                shiftadd = self.generate_fresnel_lens(f = float('inf'), xshift = x, yshift = y, result_type ='float')
                if result_type=='uint8':
                    toadd = ((img + shiftadd)%255).astype('uint8')
                    shift = np.dstack((shift,toadd))
                else :
                    toadd = (img + shiftadd)
                    shift = np.dstack((shift,toadd))
        shift = shift[:,:,1:]
        
        return shift 
        
    def create_video(self,image):
                
        sweep = np.load('C:\\Users\\LiomW17\\Desktop\\SLMfolder\\sweep.npy')
        
        for i in range(len(sweep[0,0,:])):
            sweep[:,:,i] = (sweep[:,:,i] + image)%(2*np.pi)
        
        return sweep
    
    def compute_slopes(self,shift,fslm,flens1,flens2,fobj,d1,d2,d3,WD):
        
        #fslm not used for central approximation
        
        eps= d1 + d2 +(d1*d2/flens1)
        omeg= 1.0-(d1/flens1)
        gamma=(-1.0*eps/flens2)+omeg
        
        
        slope = shift/((eps+d3*gamma)+(WD*(-1.0/fobj)*(eps+d3*gamma) + gamma))
       
        
        return slope
    
    def run_acq_video(self,stack):
        

        for i in range(len(stack[0,0,:])):
            self.display_image(stack[:,:,i])
            time.sleep(0.01)
        
        
    def findbaselensparam(self,img,focale,xshiftcm,yshiftcm):
        
        output = self.generate_fresnel_lens(f = focale, xshift = xshiftcm*(10**(-2)), yshift = yshiftcm*(10**(-2)),result_type='float')
        
        image = ((((img*255)/(2*np.pi)).astype('float') + output)%255).astype('uint8')
        
        self.display_image(image)
        
    def __del__(self):
        
        self.slm.close()
        
        
        
        
        
        
        
        
        
        
        
        
        