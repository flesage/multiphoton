

import nibabel as nib
import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import math
import matplotlib.pyplot as plt
import scipy.optimize
import functools
import time
import matplotlib.pyplot as plt
import scipy.ndimage

class classtest():
    def __init__(self):
        print 'initie'
        

    def test(self):
        self.c=np.zeros((5,5))
        self.D = np.memmap('C:\\Users\\LiomW17\\Desktop\\test\\testvalidateimage.dat',dtype='uint16',mode='w+',shape=(5,5))
        self.D[:,:]=self.c
    
    def destructeurtest(self):
        print self.c
        del self.c
        del self.D
    
    def takememmap(self):
        self.c = np.memmap('C:\\Users\\LiomW17\\Desktop\\test\\testvalidateimage.dat',dtype='uint16',mode='w+',shape=(5,5))
        
    '''
    ----------------------------------------------------------------------------------------------------------------------
    '''    
    def plane(self,x, y, params):
        a = params[0]
        b = params[1]
        c = params[2]
        z = a*x + b*y + c
        return z

    def error(self,params, points):
        result = 0
        for (x,y,z) in points:
            plane_z = self.plane(x, y, params)
            diff = abs(plane_z - z)
            result += diff**2
        return result
        
    def cross(self,a, b):
        return [a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]]    
        
        
    def planemodelling(self,datas):
        
        self.data = datas # datas in array
        #print self.data

        # regular grid covering the domain of the data
        X,Y = np.meshgrid(np.arange(0.0,len(self.data[0,:,1]), 1), np.arange(0.0,len(self.data[:,0,1]) , 1))
        #print 'X=',X
        #print 'Y=',Y
        XX = X.flatten()
        YY = Y.flatten()
        A=np.zeros((len(self.data[0,:,1])*len(self.data[:,0,1]),3))
        
        k=0
        m=0
        l=0
        for j in range(len(self.data[0,:,1])*len(self.data[:,0,1])):
                
            A[j,0]=k 
            A[j,1]=m
            A[j,2]=1.0
            k=k+1
            if k==len(self.data[:,0,1]):
                k=0
                m=m+1   
        print A
        D=self.data[:,:,1].flatten()
        print D
            
        for j in range(len(self.data[0,:,1])*len(self.data[:,0,1])):
            if D[j]==0.0:
                A=np.delete(A, (l), axis=0)
                    
                l=l-1
            l=l+1    
        E=D.tolist()
        G=Imgtesting()
        G.remove_values_from_list(E, 0.0)
        D=np.asarray(E)
        print D

        order = 1    # 1: linear, 2: quadratic
        if order == 1:
            # best-fit linear plane
    
            C,_,_,_ = scipy.linalg.lstsq(A, D)    # coefficients
    
            # evaluate it on grid
            Z = C[0]*X + C[1]*Y + C[2]
            #print Z
            
            #print C
            
            
            for i in range(len(Z[:,0])):
                for j in range(len(Z[0,:])):
                    Z[i,j] = round(Z[i,j],3)
                    
                    
            #print Z
            return Z
            # or expressed using matrix/vector product
            #Z = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)

        elif order == 2:
            # best-fit quadratic curve
            E = np.c_[np.ones(A.shape[0]), A[:,:2], np.prod(A[:,:2], axis=1), A[:,:2]**2]
            C,_,_,_ = scipy.linalg.lstsq(E, D)
    
            # evaluate it on a grid
            Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)

            for i in range(len(Z[:,0])):
                for j in range(len(Z[0,:])):
                    Z[i,j] = round(Z[i,j],3)
                    
            
            
            
            
            return Z

        '''#plot points and fitted surface
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
        ax.scatter(A[:,0], A[:,1], D[:], c='r', s=50)
        plt.xlabel('X')
        plt.ylabel('Y')
        ax.set_zlabel('Z')
        ax.axis('equal')
        ax.axis('tight')
        plt.show()
        '''
    
    
    
    
    
    
        
class Imgtesting():
    def __init__(self):
        print 'initie'
        
        
    def downloadimage(self,filocation):
        
        self.img=nib.load(filocation)
        self.imgdata= self.img.get_data()
        
        return self.imgdata
    
    def plotmeanxy(self,image):
        self.image=image
        self.Imgmeanxy=np.mean(self.image,axis=(0,1))
        return self.Imgmeanxy
        
    def testingprogram(self):
        

        # Start taking the stack
        # Step one: Move motor to top position 
        self.galvos.setFinite(True)

        self.move_motor_top(self.top)
        
        n=0
          
        time.sleep(1)
        # Step two: Start stack loop
        self.zstep=float(self.lineEdit_zstep.text())
        n_steps = int(self.lineEdit_nstack.text())
        
        for k in range(n_steps):
            # Take 2D image and saving
            print k
            
            #GalvosController.laser_power_correction(self, n, self.percentdepart)
            n=n+1
            
            self.startscan()
            # Move z motors
            #self.motors.move_dx(1)
            self.motors.move_dx(-self.zstep/1000)
            
            time.sleep(0.1)
            if self.stopped:
                break

        #time.sleep(2)
                
        self.galvos.setFinite(False)
        
        
    def remove_values_from_list(self,the_list, val):
        while val in the_list:
            the_list.remove(val)   
            
            
    
    
        
    def Maskmaking(self,filelocation,fileMIP,fileMask):
        
        img=nib.load(str(filelocation))
        vol = img.get_data()
        vol = np.squeeze(vol)
        mat = np.zeros([len(vol[:,0,0]),len(vol[0,:,0]),7])

        for i in range (60):
            for j in range(7):
                mat[:,:,j]= vol[:,:,7*i+j]
                print 'j=',j
            matmax = np.amax(mat,axis=2)
            print 'matmax'
            for k in range(7):
                vol[:,:,7*i+k] = matmax
                print 'k=',k
    

        imgsave = nib.Nifti1Image(vol, np.eye(4))
        nib.save(imgsave, str(fileMIP) + '//MIP.nii')
        
        img=nib.load( str(fileMIP) + '//MIP.nii')
        vol = img.get_data()
        vol = np.squeeze(vol)
        plt.imshow(vol[:,100,:])
        vol[vol<170] = 0
        vol[vol>0] = 100
        plt.imshow(vol[:,100,:])
        mask = scipy.ndimage.filters.gaussian_filter(vol,5)
        plt.imshow(mask[:,100,:])
        mask = mask.astype(float)
        mask[mask<16]= 0
        mask[mask>0]= 1
        plt.imshow(mask[:,100,:])


        for i in range(len(mask[0,0,:])):
            mask[:,:,i] = scipy.ndimage.morphology.binary_fill_holes(mask[:,:,i])
            mask[:,:,i] = scipy.ndimage.morphology.binary_dilation(mask[:,:,i], iterations = 10)
            mask[:,:,i] = scipy.ndimage.morphology.binary_fill_holes(mask[:,:,i])
            mask[:,:,i] = scipy.ndimage.morphology.binary_erosion(mask[:,:,i], iterations = 14)

        mask = mask.astype(float)
        mask = mask.astype(np.int16)

        imgsave = nib.Nifti1Image(mask, np.eye(4))
        nib.save(imgsave,str(fileMask) +'//mask.nii')
        
        
    def renumberingslice(self,fileslocation,filesdeposition,numberofnx,numberofny,numberofslice,numbertoadd):
        
        
        for i in range(numberofnx):
            for j in range(numberofny):
                for k in range(numberofslice):
                    try:
                        img0 =nib.load( fileslocation + '//channel_0_nx_' + str(i) + '_ny_'+ str(j) + '_slice_'+ str(k) + '.nii.gz')
                        time.sleep(0.1)
                        img1 =nib.load( fileslocation + '//channel_1_nx_' + str(i) + '_ny_'+ str(j) + '_slice_'+ str(k) + '.nii.gz')
                        time.sleep(0.1)
                        
                        nib.save(img0, filesdeposition + '//channel_0_nx_' + str(i) + '_ny_'+ str(j) + '_slice_'+ str(k+numbertoadd) + '.nii.gz')
                        time.sleep(0.1)
                        nib.save(img1, filesdeposition + '//channel_1_nx_' + str(i) + '_ny_'+ str(j) + '_slice_'+ str(k+numbertoadd) + '.nii.gz')
                        time.sleep(0.1)
                        
                    except:
                        pass
    
    
    def getPolyReturn(self,line1_VoltStart, line1_VoltEnd, n_pts_line1, line2_VoltStart, line2_VoltEnd, n_pts_line2, n_extra_pts):
        '''Returns polynomial for galvo ramp to interpolate between two ramps. This
        function solves the polynomial return so that: start and end values are
        the end of line 1 et start of line 2. Slope at the start = slope of line
        1. Slope at the end = slope of line 2.
        n_extra_pts is the number of points used in the polynomial interpolation. '''

        # Force interpretation as float to avoid issues below
        line1_VoltStart = float(line1_VoltStart)
        line2_VoltStart = float(line2_VoltStart)

        # Polynomial equation left-side
        aEqu = np.reshape(np.array([1,1,1,1]),[4,1])
        bEqu = np.reshape(np.array([1,n_extra_pts,pow(n_extra_pts,2),pow(n_extra_pts,3)]),[4,1])
        cEqu = np.reshape(np.array([0,1,2,3]),[4,1])
        dEqu = np.reshape(np.array([0,1,2*n_extra_pts,3*pow(n_extra_pts,2)]),[4,1])
        # Polynomial equation right-side
        polyEqu = np.array([line1_VoltEnd, line2_VoltStart, (line1_VoltEnd-line1_VoltStart)/n_pts_line1, (line2_VoltEnd-line2_VoltStart)/n_pts_line2])
        # Solve poly equation
        x=np.linalg.solve(np.transpose(np.hstack((aEqu, bEqu, cEqu, dEqu))),np.transpose(polyEqu))
        if n_extra_pts > 0:
            yRampReturn = np.linspace(2, n_extra_pts-1, n_extra_pts)
            y = x[0] + x[1]*yRampReturn + x[2]*pow(yRampReturn,2) + x[3]*pow(yRampReturn,3)
        else:
            y=np.empty()
        return y
    
    

    '''for name in filename:
        img = nibabel.load("C://Users//LiomW17//Desktop//Joel_dMRI//noddi_FIT_OD//"+name)
        vol = img.get_data()
        vol=vol.squeeze()
        save = nibabel.Nifti1Image(vol,numpy.eye(4))
        nibabel.save(save,"C://Users//LiomW17//Desktop//Joel_ANOVA//noddi_FIT_OD//"+name[:8]+".nii")'''