import numpy as np
import open3d as o3d
import Ransac as rc
import copy
import utility as U
from collections import defaultdict

class Plane(object):
    def __init__(self,pointCloud,equation,color):
        self.pcd=pointCloud
        self.normal=equation[:3]
        self.d=equation[3]
        self.color=color
        self.parallel_axis()
    def parallel_axis(self,plane_threshold=0.999):
        self.x=abs(U.vector_cos(self.normal,np.array([1,0,0])))>plane_threshold
        self.y=abs(U.vector_cos(self.normal,np.array([0,1,0])))>plane_threshold
        self.z=abs(U.vector_cos(self.normal,np.array([0,0,1])))>plane_threshold

        self.y_perp=abs(U.vector_cos(self.normal,np.array([0,1,0])))<0.1
    def __contains__(self,point,threshold=0.008):
        """returns true if poiint in plane , false else"""
        return abs(np.dot(point,self.normal)+self.d)<threshold
    
    def add_points(self, points):
        if len(points) == 0:
            return        
        existing_points = np.asarray(self.pcd.points)
        all_points = np.vstack((existing_points, points))
        self.pcd.points = o3d.utility.Vector3dVector(all_points)
        self.pcd.paint_uniform_color(self.color)

    
    def __str__(self):
        return str(self.normal)+" " +str(self.d)+" with "+str(np.asarray(self.pcd.points).shape[0])
class PlaneDetection:
    def __init__(self,pcd,num_of_planes) -> None:
        self.pcd=pcd
        self.num_of_planes=num_of_planes
        self.planes=[]
        self.Ransac=rc.Ransac()
        


        self.post_processing=True
        self.max=pcd.get_max_bound()
        self.min=pcd.get_min_bound()

        self.plane_normals=[]
        self.objects=[]
    def find_all_planes(self,wall_thickness):
        for i in range(self.num_of_planes):
            points = np.asarray(self.pcd.points)

            # Perform RANSAC plane segmentation

            best_plane, best_inliers = self.Ransac.ransac_plane_segmentation(points, distance_threshold=wall_thickness,num_iterations=4000)

            # Print the best plane equation
            [a, b, c, d] = best_plane
            print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

            inlier_cloud = self.pcd.select_by_index(best_inliers)

            self.pcd = self.pcd.select_by_index(best_inliers, invert=True)
            color=np.random.rand(3)
            inlier_cloud.paint_uniform_color(color) 
            self.planes.append(Plane(inlier_cloud,np.array([a,b,c,d]),color=color))
            self.plane_normals.append((inlier_cloud,np.array([a,b,c,d])))

        if self.post_processing:
            self.vert_planes=[]
            remove=[]
            for plane in self.planes:
                if plane.y:
                    self.vert_planes.append(plane)
                elif  not plane.y_perp:
                    self.objects.append(plane.pcd)
                    remove.append(plane)
            
            for plane in remove:
                self.planes.remove(plane)
            
            
           
            max=-np.inf
            min=np.inf
            ptr_max=-1
            ptr_min=-1
            for i in range(len(self.vert_planes)):

                plane=self.vert_planes[i]
                normal=plane.normal
                b=normal[1]
                d=plane.d
                height=-d/b
                if height>max:
                    max=height
                    ptr_max=i
                    
                if height<min:
                    min=height
                    ptr_min=i
            for i in range(len(self.vert_planes)):
               
                if  not (i==ptr_min or i==ptr_max):
                    self.objects.append(self.vert_planes[i].pcd)
                    self.planes.remove(self.vert_planes[i])
                    
                    
            objectsPcd=copy.copy(self.pcd)
            points=np.asarray(objectsPcd.points)
            for i in range(len(self.objects)):
                points=np.vstack((points,np.asarray(self.objects[i].points)))
            objectsPcd.points=o3d.utility.Vector3dVector(points)
            self.objects=objectsPcd

            fix_objects=True
            if fix_objects:

                deletion = []
                # Iterate over each plane
                for k, plane in enumerate(self.planes):
                    # Iterate over each point in the object point cloud
                    points = np.asarray(self.objects.points)
                    
                    for j in range(points.shape[0]):
                        point = points[j, :]
                        # Check if the point belongs to the current plane
                        if point in plane:  # Use the __contains__ method of Plane
                            # print(plane)
                            self.planes[k].add_points(point)
                            deletion.append(j)
                    # Add indices of points to be deleted to the deletion list
                    # deletion.extend(points_to_del)

                # Remove points from object and update the point cloud
                points = np.asarray(self.objects.points)
                mask = np.ones(len(points), dtype=bool)
                mask[deletion] = False  # Mark points to be deleted as False
                self.objects.points = o3d.utility.Vector3dVector(points[mask])

            
    def getPlanes(self):
        return [plane.pcd for plane in self.planes]
    def getObjects(self):
        
        objectsPcd=copy.deepcopy(self.objects)
        if self.post_processing:
            points=np.asarray(objectsPcd.points)
            #Keep Points under some constant in y axis
            points=points[points[:,1]<(self.max[1]+self.min[1])/2]
            objectsPcd.points=o3d.utility.Vector3dVector(points)
        objectsPcd.paint_uniform_color([0,0,0])

        return objectsPcd
