import open3d as o3d
import numpy as np
import multiprocessing
from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)
from graph import Point

def o3d_to_vvr(data):
    if isinstance(data, o3d.geometry.PointCloud):
        pcd=PointSet3D()
        pcd.points=np.asarray(data.points)
        pcd.colors=np.asarray(data.colors)
        return pcd
    elif isinstance(data, o3d.geometry.TriangleMesh):
        mesh=Mesh3D(color=Color.GRAY)
        mesh.vertices=np.asarray(data.vertices)
        mesh.triangles=np.asarray(data.triangles)
        if np.asarray(data.vertex_colors).shape[0]>0 :
            mesh.color=np.asarray(data.vertex_colors)[0]
        return mesh
    elif isinstance(data, o3d.geometry.LineSet):
        lineset=LineSet3D()
        lineset.points=np.asarray(data.points)
        lineset.lines=np.asarray(data.lines)
        lineset.colors=np.asarray(data.colors)
        return lineset
def list_to_graph(obstacle_list):
    polygons=[]
    for obst in obstacle_list:
        polygon=[]
        for point in obst:
            polygon.append(Point(point[0],point[1]))
        polygons.append(polygon)
    return polygons
def graph_to_list(graph_nodes_list):
    return [np.array([point.x,point.y]) for point in graph_nodes_list]