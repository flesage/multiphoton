'''
Created on Nov 18, 2015

@author: flesage
'''

import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate

if __name__ == '__main__':
    a=np.linspace(0,1024,1024)
    im=np.transpose(np.tile(a,[512,1]))
    print im.shape
    x1=a
    x2=np.copy(a)
    x2[0:500]=np.linspace(1,2,500)

    # Build interpolation matrix
    interp_matrix = np.zeros((1024,1024))
    for i in range(1024):
        tmp=np.zeros((1,1024))
        tmp[0,i]=1.0
        f=scipy.interpolate.interp1d(x1,tmp)
        interp_matrix[:,i]=f(x2)

    im2=np.dot(interp_matrix,im)

    plt.figure()
    plt.imshow(im)
    plt.figure()
    plt.imshow(im2)
    plt.show()