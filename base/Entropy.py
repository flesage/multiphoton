
import numpy as np
from scipy.interpolate import interp1d

def getInformationEntropy(vol, nbins=64):
    Hnorm, binEdges = np.histogram(np.ravel(vol), nbins, density=True)
    Hnorm = np.insert(Hnorm, 0, Hnorm[0]) # binEdges : intervalles 
    P = interp1d(binEdges, Hnorm)
    prob = P(np.ravel(vol))
    entropy = - np.sum(prob*np.log(prob))/np.log(vol.size)
    return entropy