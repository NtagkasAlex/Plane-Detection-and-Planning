import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from open3d.visualization.gui import MouseEvent, KeyEvent
from open3d.visualization.rendering import Camera

from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)
import mikowski
import numpy as np
import utility as U
import copy
import os 
import matplotlib.pyplot as plt
import PlaneDetection as PD
import PlaneDetectionMesh as PDM
import multiprocessing as mp
import DBSCAN as db
import DoorDetection as DD
import conversions as C
from sample import Sampler
from PathPlanning import clustering_objects,get_vis_graph
from graph import Point
from DBSCAN import dbscan 

class Room(object):
    def __init__(self,name:str,compute=False):
        current_file_directory = os.path.dirname(os.path.abspath(__file__))
        parent_directory = os.path.dirname(current_file_directory)
        target_folder = os.path.join(parent_directory, name)

        self.name=target_folder

        self.planes=None
        self.objects=None
        self.door=None
        self.vgraph=None
        self.min=None
        self.max=None
        mesh=o3d.io.read_triangle_mesh(self.name+str("/mesh.ply"))

        self.mesh=copy.deepcopy(mesh).translate((-15, 0, -10))
        self.mesh.compute_triangle_normals()

        if compute:
            self.compute()
        else:
            self.load()
        
    def load(self):
        self.planes=U.load_planes(self.name+str("/Planes"))
        self.objects=o3d.io.read_point_cloud(self.name+"/objects.ply")
        self.get_bound()
    def cluster_objects(self,eps=0.2,min_points=55):
        
        pcd_copy=copy.copy(self.objects)
        points=np.asarray(pcd_copy.points)
        clusters=dbscan(points,eps,min_points)
        colors = np.zeros((points.shape[0], 3))
        unique_labels = set(clusters)
        for label in unique_labels:
            if label == -1:
                colors[clusters == label] = [0, 0, 0] 
            else:
                random_color = np.random.rand(3)
                for i in range(colors.shape[0]):
                    if clusters[i]==label:
                        colors[i]=random_color
        pcd_copy.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd_copy
    def save(self):
        U.delete_contents(self.name)
        for i in range(len(self.planes)):
            o3d.io.write_point_cloud(self.name+"/Planes/planes"+str(i)+".ply",self.planes[i])

        o3d.io.write_point_cloud(self.name+"/objects.ply",self.objects)
    def sample(self,density=500):
        samp=Sampler(self.mesh)
        pcd=samp.sample(density)
        return pcd
    def compute(self,density=500,n_planes=7,wall_thickness=0.03):
        samp=Sampler(self.mesh)
        pcd=samp.sample(density)
        pd=PD.PlaneDetection(pcd,n_planes)
        pd.find_all_planes(wall_thickness)
        self.planes=pd.getPlanes()
        self.objects=pd.getObjects()
        self.get_bound()
        

    def get_bound(self):
        self.min_objects=self.objects.get_min_bound()
        self.max_objects=self.objects.get_max_bound()
        self.min_room=self.mesh.get_min_bound()
        self.max_room=self.mesh.get_max_bound()

    def build_all(self):
        if self.door is None:
            self.build_door()
        self.build_vgraph()

        if self.vgraph is None:
            self.build_vgraph()

        # self.door.show_doorway()
    def build_door(self):
        self.door=DD.DoorDetection(max_bound=self.max_room,min_bound=self.min_room)
        self.door.planes=self.planes
        self.door.clustering()
        self.door.get_doors_sized()
        print(self.door.doors)
        if len(self.door.doors)>0:
            self.doorway=self.door.show_doorways()
        self.door_targets=self.targets()
        print(self.door_targets)
        # self.door.get_doors()
        # self.doorway=self.door.show_doorway()
    
        # self.door_point=self.calculate_door_points()
    def targets(self):
        targets=self.door.doors
        if len(targets)==0:
            return np.array([0,0,0])
        return np.mean(targets,axis=1)
   

    def build_vgraph(self):
        object_pcd=copy.copy(self.objects)
        pcd_dbscan,pts_by_cluster=clustering_objects(objects_pcd=object_pcd,project=True)
        vgraph,obst=get_vis_graph(pcd_dbscan,pts_by_cluster)
        self.vgraph=vgraph
        self.obstacles=obst

    def get_short_path(self,start_point,end_point):
        if len(start_point)==3:
            start_point=Point(start_point[0],start_point[2])
            end_point=Point(end_point[0],end_point[2])
        else:
            start_point=Point(start_point[0],start_point[1])
            end_point=Point(end_point[0],end_point[1]) 

        path=self.vgraph.shortest_path(start_point, end_point,(self.min_objects,self.max_objects))
        self.path=[np.array([point.x,point.y]) for point in path]
        return self.path
    def __hash__(self):
        return hash(self.name)
    def __contains__(self,point):
        if len(point) == 2:
            point=np.array([point[0],self.min_room[1],point[2]])
            
        return point[0]<=self.max_room[0] and point[0]>=self.min_room[0] and point[2]<=self.max_room[2] and point[2]>=self.min_room[2] 



