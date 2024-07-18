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
import argparse
defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"
unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5

import matplotlib.pyplot as plt
import os
import Obstacles 
from sample import Sampler
from PathPlanning import get_vis_graph,clustering_objects
from graph import Point

from room import Room
from HighLevelPlanner import highLevelPlanner


WIDTH = 1200
HEIGHT = 700

class AppMultiRoom(Scene3D):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, "Lab6", output=True, n_sliders=3)
        self.n_planes=100
        self.reset_mesh()
        self.reset_sliders()
        self.printHelp()
        
    # def load_room(self,name):
        # self.name=name
    def reset_mesh(self):
        
        self.names=["room4","room5","room6","room7"]
        self.rooms=[Room(name,False) for name in self.names]

        self.meshes=[C.o3d_to_vvr(room.mesh) for room in self.rooms]
        self.wireframes = [LineSet3D.create_from_mesh(_mesh) for _mesh in self.meshes]

        ##remove
        for i in range(4):
            self.removeShape("wireframe"+str(i))

        self.removeShape("mesh")
        self.removeShape("obj")
        self.removeShape("pcd")
        for i in range(self.n_planes):
            self.removeShape("plane"+str(i))
        ##add
        for i,room in enumerate(self.rooms):
            self.addShape(C.o3d_to_vvr(room.mesh),"mesh"+str(i))
        for i in range(4):
            self.addShape(self.wireframes[i],"wireframe"+str(i))

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
        self.set_slider_value(1, 0.5)
        self.set_slider_value(2, 0.5)

        
    @world_space
    def on_mouse_press(self, x, y, z, button, modifiers):
        if button == Mouse.MOUSELEFT and modifiers & Key.MOD_SHIFT:
            if np.isinf(z):
                return
            self.removeShape("start")

            self.start=Point3D((x,y,z),color=(1,0,0),size=3)
            self.start_point=np.array([x,y,z])
            self.addShape(self.start,"start")
        if button == Mouse.MOUSERIGHT and modifiers & Key.MOD_SHIFT:
            if np.isinf(z):
                return
            self.removeShape("end")

            self.end=Point3D((x,y,z),color=(0,0,1),size=3)
            self.end_point=np.array([x,y,z])
            self.addShape(self.end,"end")

    def on_key_press(self, symbol, modifiers):

        if symbol == Key.R:
            self.reset_mesh()
        #Show Floor
        if symbol == Key.P:
            self.room.compute(density=self.density,n_planes=self.n_planes_slider,wall_thickness=self.wall_thickness)
            self.planes=self.room.planes
            self.n_planes=len(self.planes)
            self.objects=self.room.objects
        if symbol == Key.A:
            planes,objs=planeDetectionMesh(self.room.mesh,self.n_planes_slider)
            self.planes=planes
            self.n_planes=len(planes)        
            self.objects=objs
        
        if symbol == Key.S:
            self.room.save()
        if symbol == Key.F:
            min,max=self.find_global_min_max()
        
            pts=np.array([[min[0],min[1],min[2]],
                          [min[0],min[1],max[2]],
                          [max[0],min[1],max[2]],
                          [max[0],min[1],min[2]]])
            floor=U.get_plane(np.array(pts))
            self.addShape(floor,"floor")
        if symbol == Key. M :
            sampled_pcd=self.room.sample(self.density)
            self.addShape(C.o3d_to_vvr(sampled_pcd),"pcd")
        if symbol == Key.D:
            for i,room in enumerate(self.rooms):
                print(i)
                room.build_door()

        if symbol == Key.B:
            self.outlines={}
            self.minkowski={}
            self.n_obstacles={}
            
            for i,room in enumerate(self.rooms):
                room.build_all()
                self.outlines[i]=room.obstacles.show_outlines()
                self.minkowski[i]=room.obstacles.show_min()
                self.n_obstacles[i]=len(self.minkowski[i])
                
        if symbol == Key.L:
            self.planes={}
            self.objects={}
            self.rooms=[]
            for i in range(len(self.names)): 
                name=self.names[i]
                room=Room(self.names[i],False)
                self.planes[name]=room.planes
                self.objects[name]=room.objects
                self.rooms.append(room)

        if symbol == Key.C:
            self.clustered_objects=self.room.cluster_objects()
        if symbol == Key.O:
            ####TODO ADD High level Planner here
            ## given start and end point find the path between them
            planner=highLevelPlanner(self.rooms)
            planner.build()
            path=planner.short_path(self.start_point,self.end_point)
            line_path=U.show_path(np.array(path))
            self.removeShape("path")
            self.addShape(C.o3d_to_vvr(line_path),"path")

        #TOGGLES
        if symbol == Key._1:
            if self.show_planes:
                for i,name in enumerate(self.names):
                    for j,plane in enumerate(self.planes[name]):
                        self.removeShape(name+"plane"+str(j))
                self.show_planes=False
            else:
                for i,name in enumerate(self.names):
                    for j,plane in enumerate(self.planes[name]):
                        self.addShape(C.o3d_to_vvr(plane),name+"plane"+str(j))
                self.show_planes=True

        if symbol == Key._2:
            if self.show_mesh:
                for i,mesh in enumerate(self.meshes):
                    self.removeShape("mesh"+str(i))
                    self.removeShape("wireframe"+str(i))
                self.show_mesh=False
            else:
                for i,mesh in enumerate(self.meshes):
                    self.addShape(mesh,"mesh"+str(i))
                    self.addShape(self.wireframes[i],"wireframe"+str(i))
                self.show_mesh=True
        if symbol == Key._3:
            if self.show_objects:
                for i,name in enumerate(self.names):
                    self.removeShape("obj"+str(i))
                self.show_objects=False
            else:
                for i,name in enumerate(self.names):
                    self.addShape(C.o3d_to_vvr(self.objects[name]),"obj"+str(i))
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
                for i,name in enumerate(self.names):
                    self.removeShape("door"+str(i))
                self.show_door=False
            else:
                for i,name in enumerate(self.names):
                    self.addShape(C.o3d_to_vvr(self.rooms[i].doorway),"door"+str(i))
                self.show_door=True
        if symbol == Key._6:
            return
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
                for i in range(len(self.names)):
                    for j in range(self.n_obstacles[i]):
                        self.removeShape(str(i)+"outline"+str(j))
                        self.removeShape(str(i)+"min"+str(j))
                self.show_obstacles=False
            else:
                for i in range(len(self.names)):
                    for j in range(self.n_obstacles[i]):
                        self.addShape(C.o3d_to_vvr(self.outlines[i][j]),str(i)+"outline"+str(j))
                        self.addShape(C.o3d_to_vvr(self.minkowski[i][j]),str(i)+"min"+str(j))
                        
                    
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
    def find_global_min_max(self):
        
        global_min = np.copy(self.rooms[0].min_room)
        global_max = np.copy(self.rooms[0].max_room)

        for room in self.rooms:
            global_min = np.minimum(global_min, room.min_room)
            global_max = np.maximum(global_max, room.max_room)

        return global_min, global_max
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
        SHIFT+M2: Select End Point\n\
        R: Reset mesh\n\
        B: Build Door and Visibillity Graph\n\
        L: Load Rooms\n\
        O: Find Path from Start to End\n\
        1: Toggle planes\n\
        2: Toggle Mesh\n\
        3: Toggle Objects\n\
        5: Toggle Doors\n\
        7: Toggle Outlines\n\
        8: Toggle Visibillity\n\
        ?: Show this list\n\n")
def planeDetectionMesh(mesh,number_of_planes):
    pd=PDM.PlaneDetection(mesh,number_of_planes)
    pd.find_all_planes()
    planes=pd.getPlanes()
    obj=pd.getObjects()
    
    return planes,obj
    
if __name__=="__main__":
    

    app = AppMultiRoom()
    
    app.mainLoop()
    