import open3d as o3d
import numpy as np
class Ransac:
    def __init__(self) -> None:
        pass
    def fit_plane(self,points):
        """
        Fit a plane to a set of points using least squares.

        - points: 3x3 array of points.

        Returns:
        - plane coef [a, b, c, d] 

        """ 
        v1=points[0,:]
        v2=points[1,:]
        v3=points[2,:]
        normal=np.cross(v2-v1,v3-v1)
        #colinear case
        if (normal==0).all():
            eps=1e-5
            return [eps,eps,eps,1]    
        
        return [normal[0], normal[1], normal[2], -np.dot(normal,v1)]
        

    def ransac_plane_segmentation(self,points, distance_threshold=0.005, num_iterations=5000):
        """
        Perform RANSAC to segment the largest plane from the point cloud.

        Parameters:
        - points (numpy.ndarray): Nx3 array of point cloud data.
        - distance_threshold (float): Maximum distance a point can be from the plane to be considered an inlier.
        - num_iterations (int): Number of RANSAC iterations.

        Returns:
        - best_plane (list): Coefficients of the best plane equation [a, b, c, d].
        - best_inliers (list): Indices of the inlier points.
        """
        best_plane = None
        best_inliers = []
        num_points = points.shape[0]

        for _ in range(num_iterations):
            # Sample 3 point from the range of points
            
            sample_indices = np.random.randint(0, num_points,size=3)
            
            sample_points = points[sample_indices, :]
            
            # Fit a plane to these 3 points with Least Squares ( or other method)
            plane = self.fit_plane(sample_points)
            # print(np.dot(np.array(plane[:3]),sample_points[0,:])+plane[3])
            # Calculate the distance of all points to the plane 
            # using d=|ax_0+by_0+cz_0+D|/sqrt(a^2+b^2+c^2)
            distances = np.abs(np.dot(points, plane[:3]) + plane[3])/np.sqrt(plane[0]**2+plane[1]**2+plane[2]**2)
            # And find where this distance is smaller thatn the threshold
            inliers = np.where(distances <= distance_threshold)[0]
            #if inliears of this plane are more than the prev best we keep the current else continue
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_plane = plane

        return best_plane, best_inliers

        