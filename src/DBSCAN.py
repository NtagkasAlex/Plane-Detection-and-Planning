import numpy 
import open3d as o3d
import KDTree 
def dbscan(points, eps, MinPts):
    '''
    points: NDarray with the pcd points

    eps: area in which we look for neibours
    
    MinPts : Minimum points to make a cluster.
    
    Returns :
    List of clusters -1 is noise , anything else is good   
    '''
 
    #init as 0  
    clusters = [0]*points.shape[0]
    root=KDTree.KdNode.build_kd_node(points)
    # C is the ID of the current cluster.    
    C = 0
    for i in range(0, points.shape[0]):
        #Only traverse points that dont belong in a cluster already
        if not (clusters[i] == 0):
           continue
        # print(points.shape[0]-i)
        # Find all  neighboring points.
        NeighborPts = region_query(root,points, i, eps)
        
        
        # print(points.shape[0]-i)
        #If it has less that MinPts Neighbors then its noise
        if len(NeighborPts) < MinPts:
            
            clusters[i] = -1
        # Otherwise, if there are at least MinPts , use this point as the 
        # seed for a new cluster.    
        else: 
           C += 1
           grow_cluster(root,points, clusters, i, NeighborPts, C, eps, MinPts)
    
    return clusters


def grow_cluster(tree_root,points, clusters, idx, NeighborPts, C, eps, MinPts):
    '''
    
    Create a new cluster based on the points with index "idx"
      
    '''

    clusters[idx] = C
    
    #NeighborPts is used as a Queue
    i = 0
    while i < len(NeighborPts):    
        # Get the next point from the queue.        
        i_neibour = NeighborPts[i]

        #If declared noise at first add to cluster
        if clusters[i_neibour] == -1:
           clusters[i_neibour] = C
        
        # Otherwise, if Pn isn't already claimed, claim it as part of C.
        elif clusters[i_neibour] == 0:
            # Add Pn to cluster C (Assign cluster label C).
            clusters[i_neibour] = C
            
            # Find all the neighbors of this i_neibour
            iNeighborPts = region_query(tree_root,points, i_neibour, eps)
            
            # If Pn has at least MinPts neighbors, it's a branch point!
            # Add all of its neighbors to the FIFO queue to be searched. 
            if len(iNeighborPts) >= MinPts:
               
                NeighborPts.extend(iNeighborPts)
        
        i += 1        
    

def region_query(tree_root,points, p_idx, eps):
    '''
    All points within a range "eps" of "point[p_idx]" 
    '''    
    
    pts_idxs = KDTree.KdNode.inSphere(points[p_idx],eps,tree_root) 
    
    return pts_idxs

