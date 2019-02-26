import numpy as np
import nibabel as nib
import numpy as np
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import math
import matplotlib.pyplot as plt
import scipy.optimize
import functools



class planefitting():

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
            #print A
            D=self.data[:,:,1].flatten()
            #print D
            
            for j in range(len(self.data[0,:,1])*len(self.data[:,0,1])):
                if D[j]==0.0:
                    A=np.delete(A, (l), axis=0)
                    
                    l=l-1
                l=l+1    
            E=D.tolist()
            self.remove_values_from_list(E, 0.0)
            D=np.asarray(E)
            #print D

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
                    
            
            
            
                #plot points and fitted surface
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
                return Z

            
            
       
    def remove_values_from_list(self,the_list, val):
        while val in the_list:
            the_list.remove(val)   
            