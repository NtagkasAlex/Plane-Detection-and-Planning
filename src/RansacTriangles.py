import open3d as o3d
import numpy as np
import utility as U
class RansacTriangles:
    def __init__(self,mesh) -> None:
        self.mesh=mesh
        self.triangles=np.asarray(self.mesh.triangles)
        self.vertices=np.asarray(self.mesh.vertices)
        self.normals=np.asarray(self.mesh.triangle_normals)
    def fit_plane(self,triangle_idx):
        
        normal=self.normals[triangle_idx,:].reshape(3)
        p1=self.vertices[self.triangles[triangle_idx,0],:]
        d=-np.dot(normal,p1.reshape(3))

        return [normal[0], normal[1], normal[2], d]
        

    def ransac_plane_segmentation(self,mesh, threshold=0.01, num_iterations=5000):
        """
        Perform RANSAC to segment the largest plane from the point cloud.

        Parameters:
        - points (numpy.ndarray): Nx3 array of point cloud data.
        - distance_threshold (float): Maximum distnormalsance a point can be from the plane to be considered an inlier.
        - num_iterations (int): Number of RANSAC iterations.

        Returns:
        - best_plane (list): Coefficients of the best plane equation [a, b, c, d].
        - best_inliers (list): Indices to triangles on the best plane
        """
        self.__init__(mesh)
        best_plane = None
        best_area=-np.inf
        best_inliers = []
        num_triangles = self.triangles.shape[0]

        for _ in range(num_iterations):
            
            triangle_index = np.random.randint(0, num_triangles,size=1)
            
            plane = self.fit_plane(triangle_index) 
            # print(np.dot(np.array(plane[:3]),sample_points[0,:])+plane[3])
            # Calculate the distance of all points to the plane 
            # And find where this distance is smaller thatn the threshold
            # inlier_triangles =self.triangles[self.normals]
            vec=U.vector_cos(self.normals,np.array(plane[:3]))
            # print(vec)
            # print(vec[abs(np.dot(self.vertices[self.triangles[:,0],:],np.array(plane[:3]))+plane[3])<0.01 & abs(vec)>0.99])
            inliers=np.where((abs(np.dot(self.vertices[self.triangles[:,0],:],np.array(plane[:3]))+plane[3])<threshold) & (abs(vec)>1-threshold))[0]
            # print(np.where(abs(vec)>0.99))
            inlier_triangles=self.triangles[inliers]
            inlier_normals=self.normals[inliers]
            area=1/2*np.linalg.norm(inlier_normals,axis=1)
            area=np.sum(area)
            if area>best_area:
                best_area=area
                best_inliers=inliers
                best_plane=plane

        return best_plane, best_inliers

        