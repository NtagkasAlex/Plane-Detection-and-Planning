import numpy as np
import open3d as o3d
import Ransac as rc
import utility as U
from multiprocessing import Pool

def uniform(density,vertices,triangles):
    points=[]
    for i in range(triangles.shape[0]):
        v1=vertices[triangles[i,0],:]
        v2=vertices[triangles[i,1],:]
        v3=vertices[triangles[i,2],:]
        area=U.triangle_area(v1,v2,v3)
        ratio=(int) (area*density)
        for _ in range(ratio):
            a=np.random.rand()
            b=np.random.rand()
            
            sample=(1-np.sqrt(a))*v1+np.sqrt(a)*(1-b)*v2+np.sqrt(a)*b*v3
            points.append(sample)
    return points

class Sampler():
    def __init__(self,mesh):
        self.mesh=mesh
        self.vertices = np.asarray(mesh.vertices)
        self.tri=np.asarray(mesh.triangles)
        self.num_triangles=self.tri.shape[0]
    def sample(self,density):
        workers=8
        pool = Pool(workers)
        batch_size=30
        batches = [(density,self.vertices, self.tri[i:i + batch_size])
                        for i in range(0, self.num_triangles, batch_size)]

        results = list(pool.starmap(uniform, batches))
        results=  [item for sublist in results if sublist for item in sublist]
        # print(len(results))
        points=np.stack(results)
        pcd=o3d.geometry.PointCloud()
        pcd.points=o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0,0,0])
        return pcd


        