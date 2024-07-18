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
import argparse
defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"
unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5

import Obstacles 
from PathPlanning import clustering_objects
from room import Room

WIDTH = 1200
HEIGHT = 700

class AppSingleRoom(Scene3D):
    def __init__(self,name):
        super().__init__(WIDTH, HEIGHT, "Lab6", output=True, n_sliders=3)
        self.name=name
        self.n_planes=100
        self.reset_mesh()
        self.reset_sliders()
        self.printHelp()
        

    def reset_mesh(self):

        self.room=Room(self.name,False)
        self.mesh3d=C.o3d_to_vvr(self.room.mesh)
        self.wireframe = LineSet3D.create_from_mesh(self.mesh3d)

        ##remove
        self.removeShape("wireframe")
        self.removeShape("mesh")
        self.removeShape("obj")
        self.removeShape("pcd")
        for i in range(self.n_planes):
            self.removeShape("plane"+str(i))
        ##add
        self.addShape(self.mesh3d,"mesh")
        self.addShape(self.wireframe, "wireframe")
        self.show_mesh=True

        self.show_planes = False
        self.show_objects=False
        self.show_clustered_objects=False
        self.show_door=False
        self.show_projected_walls=False
        self.show_obstacles=False
        self.show_vis=False
        self.show_path=False


    def reset_sliders(self):
        self.set_slider_value(0, 0.5)
        self.set_slider_value(1, 0.7)
        self.set_slider_value(2, 0.25)

        
    @world_space
    def on_mouse_press(self, x, y, z, button, modifiers):
        if button == Mouse.MOUSELEFT and modifiers & Key.MOD_SHIFT:
            if np.isinf(z):
                return
            self.removeShape("start")

            self.start=Point3D((x,y,z),color=(1,0,0))
            self.start_point=np.array([x,y,z])
            self.addShape(self.start,"start")
    def on_key_press(self, symbol, modifiers):

        if symbol == Key.R:
            self.reset_mesh()
        #Show Floor
        if symbol == Key.P:
            self.print("Computing Planes...")
            self.room.compute(density=self.density,n_planes=self.n_planes_slider,wall_thickness=self.wall_thickness)
            self.planes=self.room.planes
            self.n_planes=len(self.planes)
            self.objects=self.room.objects
        if symbol == Key.A:
            self.print("Computing Planes from Mesh...")

            planes,objs=planeDetectionMesh(self.room.mesh,self.n_planes_slider)
            self.planes=planes
            self.n_planes=len(planes)        
            self.objects=objs
        
        if symbol == Key.S:
            self.room.save()
        if symbol == Key.F:
            min=self.room.min_room
            max=self.room.max_room
            pts=np.array([[min[0],min[1],min[2]],
                          [min[0],min[1],max[2]],
                          [max[0],min[1],max[2]],
                          [max[0],min[1],min[2]]])
            floor=U.get_plane(np.array(pts))
            self.addShape(floor,"floor")
        if symbol == Key. M :
            self.print("Sampling with density "+str(self.density))

            sampled_pcd=self.room.sample(self.density)
            self.addShape(C.o3d_to_vvr(sampled_pcd),"pcd")
        if symbol == Key.D:
            self.print("Building Door...")
            self.room.build_door()
            self.targets=self.room.targets()
            # for i in range(len(self.targets)):
                # self.addShape( Point3D(self.targets[i],color=(0,1,0)),"target"+str(i))
        if symbol==Key.Q:
            object_pcd=copy.copy(self.room.objects)
            pcd_dbscan,pts_by_cluster=clustering_objects(objects_pcd=object_pcd,project=True)

            obstacles=Obstacles.Obstacles(pts_by_cluster)
            obstacles.get_outline()
            obstacles.minkowski()
            self.room.obstacles=obstacles
            self.outlines=self.room.obstacles.show_outlines()
            self.minkowski=self.room.obstacles.show_min()
            self.n_obstacles=len(self.outlines)

        if symbol == Key.B:
            self.print("Building Door and Visibillity...")

            self.room.build_all()
            self.outlines=self.room.obstacles.show_outlines()
            self.minkowski=self.room.obstacles.show_min()
            self.n_obstacles=len(self.outlines)
            self.targets=self.room.targets()
            for i in range(len(self.targets)):
                self.addShape( Point3D(self.targets[i],color=(0,1,0)),"target"+str(i))
            self.end_point=self.targets[0]
        if symbol == Key.L:
            self.print("Loaded Successfully.")
            self.room=Room(self.name,False)
            self.planes=self.room.planes
            self.n_planes=len(self.planes)
            self.objects=self.room.objects
        if symbol == Key.C:
            self.print("Clustering Objects...")
            self.clustered_objects=self.room.cluster_objects()
        if symbol == Key.O:
            # self.print("Showing: ")

            path=self.room.get_short_path(self.start_point,self.end_point)
            line_path=U.show_path(np.array(path))
            self.removeShape("path")
            self.addShape(C.o3d_to_vvr(line_path),"path")
    
        #TOGGLES
        if symbol == Key._1:
            if self.show_planes:
                for i in range(self.n_planes):
                    self.removeShape("plane"+str(i))
                self.show_planes=False
            else:
                for i in range(self.n_planes):
                    self.addShape(C.o3d_to_vvr(self.planes[i]),"plane"+str(i))
                self.show_planes=True

        if symbol == Key._2:
            if self.show_mesh:
                self.removeShape("mesh")
                self.removeShape("wireframe")
                self.show_mesh=False
            else:
                self.addShape(self.mesh3d,"mesh")
                self.addShape(self.wireframe,"wireframe")
                self.show_mesh=True
        if symbol == Key._3:
            if self.show_objects:
                self.removeShape("obj")
                self.show_objects=False
            else:
                self.addShape(C.o3d_to_vvr(self.objects),"obj")
                self.show_objects=True
        if symbol == Key._4:
            if self.show_clustered_objects:
                self.removeShape("clustered_objects")
                self.show_clustered_objects=False
            else:
                self.addShape(C.o3d_to_vvr(self.clustered_objects),"clustered_objects")
                self.show_clustered_objects=True

        if symbol == Key._5:
            if self.show_door:
                self.removeShape("door")
                self.show_door=False
            else:
                self.addShape(C.o3d_to_vvr(self.room.doorway),"door")
                self.show_door=True
        if symbol == Key._6:
            proj=o3d.geometry.PointCloud()
            for wall in self.room.door.projected_clustered_walls():
                proj+=wall
            if self.show_projected_walls:
                self.removeShape("proj")
                self.show_projected_walls=False
            else:
                self.addShape(C.o3d_to_vvr(proj),"proj")
                self.show_projected_walls=True
        if symbol == Key._7:
            if self.show_obstacles:
                for i in range(self.n_obstacles):
                    self.removeShape("outline"+str(i))
                    self.removeShape("min"+str(i))
                self.show_obstacles=False
            else:
                for i in range(self.n_obstacles):
                    self.addShape(C.o3d_to_vvr(self.outlines[i]),"outline"+str(i))
                    self.addShape(C.o3d_to_vvr(self.minkowski[i]),"min"+str(i))
                self.show_obstacles=True
        if symbol == Key._8:
            vis=U.lineset_for_vertex(self.room.vgraph.show(),reduced=True)
            if self.show_vis:
                for i in range(len(vis)):
                    self.removeShape("vis"+str(i))
                self.show_vis=False
            else:
                for i in range(len(vis)):
                    self.addShape(C.o3d_to_vvr(vis[i]),"vis"+str(i))
                self.show_vis=True

    def on_slider_change(self, slider_id, value):
    
        if slider_id == 0:
            self.density = 1000 * value
        if slider_id == 1:
            self.n_planes_slider=(int) (value*10)
        if slider_id==2:
            self.wall_thickness=0.1*value
        if slider_id==4:
            self.mu=value

    def printHelp(self):
        self.print("\
        SHIFT+M1: Select Start Point\n\
        R: Reset mesh\n\
        B: Build Door and Visibillity Graph\n\
        D: Build Door\n\
        C: Cluster Objects\n\
        L: Load Room\n\
        P: Planes from Point Cloud\n\
        A: Planes from 3D Mesh\n\
        M: Created Sampled PointCloud\n\
        O: Find Path from Start to Door\n\
        1: Toggle planes\n\
        2: Toggle Mesh\n\
        3: Toggle Objects\n\
        5: Toggle Doors\n\
        7: Toggle Outlines\n\
        8: Toggle Visibillity\n\
        Slider 1: Control Density \n\
        Slider 2: Number of Planes\n\
        Slider 3: Wall thickness \n\
        ?: Show this list\n\n")
def planeDetectionMesh(mesh,number_of_planes):
    pd=PDM.PlaneDetection(mesh,number_of_planes)
    pd.find_all_planes()
    planes=pd.getPlanes()
    obj=pd.getObjects()
    
    return planes,obj
    
if __name__=="__main__":
    
    parser = argparse.ArgumentParser(description="Run the AppSingleRoom application.")
    parser.add_argument("room_name", type=str, help="The name of the room to run the app for")
    
    args = parser.parse_args()
    
    app = AppSingleRoom(args.room_name)
    
    app.mainLoop()
    