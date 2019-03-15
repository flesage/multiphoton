#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 12 11:41:32 2017

@author: Rafat Damseh
"""
# visualization


# computation
import numpy as np
from scipy.spatial import cKDTree
from sklearn.utils import shuffle
import networkx as nx
import scipy.ndimage.morphology as morph


def connect_surg(G, nodes_to_merg, cen_nodes_to_merg, cen_med_val):
    """
    This funtion modify the topolgy of the graph 'G'.    
    
    Input:
        "G": The graph to be modified. 
        
        "nodes_to_merg": Indicies of Graph nodes that will be replaced 
        by one node. The elements in each row of 'nodes_to_merg' represent the indices of 
        a group of nodes in the Graph 'G' that will be  contracted into ine node.
        
        "cen_nodes_to_merg": Geometric postions [x,y,z] of the nodes to be added to the graph 'G'.
        Each row in 'cen_nodes_to_merg' represent the position of one node that will replace the 
        nodes with indices in the corresponding row of 'nodes_to_merg'.
    
    Output:
        "G": Modified graph.
            
    """
    nbrs_of_nbrs=[]
    new_l=[]
    connect_to_nbrs=[]
    n_v=G.number_of_nodes()
    
    for i in range(len(cen_nodes_to_merg)):
     
        # get neighbours of veticies belonging to this cluster 
        nbrs_=nodes_to_merg[i]
        nbrs_=list(set(G.nodes()).intersection(set(nbrs_)))
        
        # add new vertix of this cluster centroid 
        new_l_=n_v+i
        new_l.append(new_l_)
        G.add_node(new_l_)
        G.node[new_l_]['pos']=np.array(cen_nodes_to_merg[i])
        G.node[new_l_]['node']=False # tage the node with False
                    #assign new med_val
        G.node[new_l_]['med_val']=cen_med_val[i] 
        
        nbrs_of_nbrs_=[G.neighbors(nbrs_[j]) for j in range(len(nbrs_))]
        
        #flatten the list: "nbrs_of_nbrs_"
        nbrs_of_nbrs_= [nbrs_of_nbrs_[m][n] for m in range(len(nbrs_of_nbrs_)) for n in range(len(nbrs_of_nbrs_[m]))] 
        
        #get unique indices
        nbrs_of_nbrs_=list(set(nbrs_of_nbrs_))
        
        # retain only neighbours of neighbours that are still in the graph
        nbrs_of_nbrs_=list(set(G.nodes()).intersection(set(nbrs_of_nbrs_)))
     
        # append the neighbor of neighbours for this vertix
        nbrs_of_nbrs.append(nbrs_of_nbrs_)
        
        #build new connections
        connect_to_nbrs=[  [new_l_, nbrs_of_nbrs_[k]] for k in range(len(nbrs_of_nbrs_))]
        G.add_edges_from(connect_to_nbrs) 
        
        #return edges that are still in the graph
        G.remove_nodes_from(nbrs_)
        
    return G



def assignPixelClusters(pos):
    '''
    Assigne the current nodes in graph with closest pixel
    
    Input:
        pos: position of the current graph nodes
        
    Output:
        centroids: Geometric position for the centers of the clusters as [x,y,z].
        clusters_pos: qeometric positin of the nodes grouped in eache cluster.
        clusters_points: The indecies of points gropued in each cluster.   
        
        '''
        
    clusters_init=np.round(pos).astype(int)
    
    _, _, clusters_inverse, clusters_count=np.unique(clusters_init, 
                             axis=0, return_inverse=True, return_index=True, 
                             return_counts=True)  
    
    clusters=np.where(clusters_count>1)[0]
    clusters_points=[np.where(clusters_inverse==i)[0] for i in clusters]
    
    clusters_pos=[pos[i] for i in clusters_points]
    centroids=[np.mean(i,axis=0) for i in clusters_pos]

    return centroids, clusters_pos, clusters_points



def createGraphFromSeg(label, connect=8, portion=None):
    """
    
    """
    
    print('Create random nodes on segmentation')
    
    #### random sampling of the grid ####        
    index=np.array(np.where(label>0)).T
    
    n_points=len(index)
    index=shuffle(index)
    
    #### build graph from sampled grid ####
    G=nx.Graph()
    G.add_nodes_from(range(n_points))
    
    # assign coordinates     
    nodes=index[:n_points]
    n_nodes=G.number_of_nodes()    
    for idx, i in enumerate(nodes):
        G.node[idx]['pos']=i
        
    #### build connectivity ####

    #construct connections 
    print('Biuld KDTree . . .')
    tree = cKDTree(nodes)
    indDist=tree.query(nodes, k=connect, distance_upper_bound=10)[1]
    
    print('Assign connections to graph . . .')
    c=[]
    for idx, i in enumerate(indDist):
        indD=np.unique(i)
        cc=[[idx, j]  for j in indD
                             if j != idx and j != n_nodes]
        if cc:
            c.append(cc)
    
    #assign connections         
    connections=[j for i in c for j in i] 
    G.add_edges_from(connections)  
                
    return G


def cycleArea(corners):
    n = len(corners) # of corners
    cross= [0.0,0.0,0.0]
    for i in range(n):
        j = (i + 1) % n
        crss=np.cross(corners[i],corners[j])
        cross=cross+crss
        #nrm+=np.linalg.norm(crss)
    area = np.linalg.norm(cross) / 2.0
    return area

def cycleAreaAll(corners):
        
    if len(corners):       
        n = np.shape(corners)[1] # of corners
        cross= np.zeros((np.shape(corners)[0],np.shape(corners)[2]))
        for i in range(n):
            j = (i + 1) % n
            crss=np.cross(corners[:,i],corners[:,j])
            cross=cross+crss
            #nrm+=np.linalg.norm(crss)
        area = np.linalg.norm(cross, axis=1) / 2.0
    else:
        return 0
    return area


def checkNode(a, b, thr=0):
    
    '''
    A fucntion to check a certian node is 
    to be processec in next iteration. The condoction is based on angles 
    between the edges shared with the node.
    
    Input:
        -a: coordinate of a graph node
        -b: coordinateds of 'a' neighbours
        -thr: angle threshold 

    Output:
        True if a is to be processed and Flase other wise
    '''
    if len(b)>0:
        
        p1=b[0]
        p2=b[1:]
        
        ap1 = p1 - a
        ap2 = p2 - a
        
        x1,y1,z1=ap1
        x2,y2,z2=ap2[:,0], ap2[:,1], ap2[:,2] 
        
        dot=x1*x2+y1*y2+z1*z2
        norm1=(x1**2+y1**2+z1**2)**.5
        norm2=(x2**2+y2**2+z2**2)**.5 
        
        mask=norm2>0
        cosine_angle =  np.ones_like(norm2)   
        
        if norm1>0:
            cosine_angle[mask]=dot[mask]/(norm1*norm2[mask])    
        
        mask=cosine_angle<0
        cosine_angle[mask]=cosine_angle[mask]+1
        
        add=np.zeros_like(norm2)  
        add[mask]=np.pi/2
        
        angle = np.arccos(cosine_angle)
        angle = np.degrees(angle+add)
         
        thr1, thr2 = thr, 180-thr
        chck=(angle>thr1)&(angle<thr2)
        
        return not np.any(chck) #true if skel
    
    else:
        
        return True


def isSklNodes(p, pn, thr=0):
          
    '''
    output a boolian array with True values incidicating skeletal nodes
    '''
    
    # check nodes to process
    chck=[checkNode(i,j, thr) for i, j in zip(p, pn)]
   
    return np.array(chck) # true if  skeleton



def findNodes(G, j_only=False):
    
    nodes=[]
    ind=[]
    
    if j_only:      
        
        for i in G.nodes():
            if len(G.neighbors(i))==3:
                nodes.append(G.node[i]['pos'])
                ind.append(i)      
    else:
               
        for i in G.nodes():
            if len(G.neighbors(i))!=2:
                nodes.append(G.node[i]['pos'])
                ind.append(i)
                
    try:       
        nodes=[i.tolist() for i in nodes]
    except:
        pass

    return ind, nodes


def getDist(v):
    
    l1=[v[n,:,:] for n in range(np.shape(v)[0])] #Z-XY
    l2=[v[:,n,:] for n in range(np.shape(v)[1])] #X-ZY
    l3=[v[:,:,n] for n in range(np.shape(v)[2])] #Y-ZX
    
    d1=np.array([morph.distance_transform_edt(l1[i]) for i in range(len(l1))])
    d1=d1.transpose((0,1,2))
    d2=np.array([morph.distance_transform_edt(l2[i]) for i in range(len(l2))])
    d2=d2.transpose((1,0,2))
    d3=np.array([morph.distance_transform_edt(l3[i]) for i in range(len(l3))])
    d3=d3.transpose((1,2,0))
    
    d_=np.maximum(d1,d2);d=np.maximum(d_,d3)
    return d  

 

def numpy_fill(data, lens, s=False):
    '''
    Pad an array with different row sizes
    Input:
        data: object array
        lens: length of each row of data
        s=length of each element in the row
    '''
    lens=np.array(lens)
    
    # Mask of valid places in each row
    mask = np.arange(lens.max()) < lens[:, None]

    # Setup output array and put elements from data into masked positions
    if s: 
        out = np.zeros((mask.shape[0],mask.shape[1], s), dtype=data.dtype)
        out[mask, :] = np.concatenate(data)

    else:
        out = np.zeros((mask.shape[0],mask.shape[1]), dtype=data.dtype)
        out[mask] = np.concatenate(data)

    return out.astype(float), mask   

def fixG(G):
    
    Oldnodes=G.nodes()
    new=range(len(Oldnodes))
    mapping={Oldnodes[i]:new[i] for i in new}
    G=nx.relabel_nodes(G, mapping)
    
    return G
