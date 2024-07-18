import numpy as np
import open3d as o3d
import RansacTriangles as rsc
import copy
import utility as U
class PlaneDetection:
    def __init__(self,mesh,num_of_planes) -> None:
        self.mesh=mesh
        self.num_of_planes=num_of_planes
        self.planes=[]
        self.Ransac=rsc.RansacTriangles(mesh)
        self.planes_dict={}
        self.object=False
        self.min=mesh.get_min_bound()
        self.max=mesh.get_max_bound()
        
        self.post_processing=True
        self.plane_normals=[]
        self.objects=[]
    def update(self,mesh):
        self.mesh=mesh
        self.triangles=np.asarray(self.mesh.triangles)
        self.vertices=np.asarray(self.mesh.vertices)
        self.normals=np.asarray(self.mesh.triangle_normals)
    def find_all_planes(self):
        for i in range(self.num_of_planes):
            self.update(self.mesh)
            # Perform RANSAC plane segmentation
            best_plane, best_inliers = self.Ransac.ransac_plane_segmentation(self.mesh, threshold=0.05 ,num_iterations=1000)

            # Print the best plane equation
            [a, b, c, d] = best_plane
            print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

            plane_threshold=0.98

            vertices_idx=list(set(self.triangles[best_inliers].reshape(-1)))
      
            self.inlier_mesh = self.mesh.select_by_index(vertices_idx,cleanup=False)

            self.inlier_mesh.paint_uniform_color(np.random.rand(3))
            self.planes.append(self.inlier_mesh)
            self.plane_normals.append((self.inlier_mesh,np.array([a,b,c,d])))
            self.mesh.remove_triangles_by_index(best_inliers)


        if self.post_processing:
            self.vert_planes=[]
            for i in range(len(self.plane_normals)):
                mesh=self.plane_normals[i][0]
                plane=self.plane_normals[i][1]
                normal=plane[:3]
                if U.vector_cos(normal,np.array([0,1,0]))>plane_threshold:
                    self.vert_planes.append(self.plane_normals[i])
                
            min_y=self.min[1]
            max_y=self.max[1]
            for i in range(len(self.vert_planes)):
                pcd=self.vert_planes[i][0]
                plane=self.vert_planes[i][1]
                normal=plane[:3]
                b=normal[1]
                d=plane[3]
                height=-d/b
                if abs(height-max_y)>0.5 and abs(height-min_y)>0.5:
                    self.planes.remove(self.vert_planes[i][0])
                    self.objects.append(self.vert_planes[i][0])
      
                    
            
            
    def getPlanes(self):
        
        return self.planes
    def getObjects(self):
        self.update(self.mesh)
        remove_cell=[]
        for i in range(self.triangles.shape[0]):
            if self.vertices[self.triangles[i,0]][1]>0.5:
                remove_cell.append(i)
        object_mesh=copy.deepcopy( self.mesh)
        object_mesh.remove_triangles_by_index(remove_cell)
        self.objects.append(object_mesh)
        for obj in self.objects:
            object_mesh+=obj
        object_mesh.paint_uniform_color([0,0,0])
        return object_mesh
        
