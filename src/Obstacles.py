import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from open3d.visualization.gui import MouseEvent, KeyEvent
from open3d.visualization.rendering import Camera
import mikowski
import numpy as np
import utility as U
import copy
import os 
import matplotlib.pyplot as plt
import a_shapes
# ALPHA=0.9 for room3
ALPHA=0.3 

class Obstacles():

    def __init__(self,dictionary,size_threshold=500) :
        self.points_by_obstacle=dictionary
        self.size_threshold=size_threshold
    def get_outline(self):
        self.outlines=[]
        sizes=self.sizes()
        for i in range(len(self.points_by_obstacle)-1):#-1 to not consider noise

            pts=self.points_by_obstacle[i]
            points=np.delete(pts,1,1)
            
            if sizes[i]<self.size_threshold:
                hull_indices=U.CH_graham_scan(points)
                # Points forming the convex hull
                convex_hull_points = points[hull_indices]
                outline_points=U.convert_2d_to_3d(convex_hull_points)
                self.outlines.append(outline_points)
            else:
                a=np.array(a_shapes.get_outline(points,ALPHA))
                outline_points=U.convert_2d_to_3d(a)
                self.outlines.append(outline_points)
    def sizes(self):
        return [self.points_by_obstacle[label].shape[0] for label in self.points_by_obstacle ]

    def show_outlines(self):
        return [U.create_poly_3d(x) for x in self.outlines]
    def minkowski(self):
        # self.min=[a_shapes.minkowski_sum(np.delete(x,1,1),np.array([0,0]),0.1) for x in self.outlines]
        self.min=[]
        sizes=self.sizes()
        for i in range(len(self.points_by_obstacle)-1):#-1 to not consider noise which has label -1
            if sizes[i]<self.size_threshold:
                min_points=mikowski.minkowski_sum(np.delete(self.outlines[i],1,1),np.array([0,0]),0.1)
                # min_points=mikowski.minkowski_sum_test(np.delete(self.outlines[i],1,1))


            else:
                min_points=mikowski.minkowski_sum_non_convex(np.delete(self.outlines[i],1,1),np.array([0,0]),0.1)

            if sizes[i]<self.size_threshold:

                hull_ind = U.CH_graham_scan(min_points)
                min_points = np.array(min_points)[hull_ind]
            else:
                min_points=np.array(a_shapes.get_outline(min_points,ALPHA))
            self.min.append(min_points)

   
    def show_min(self):
        return [U.create_poly_3d(x,color=[1,0,0]) for x in self.min]
