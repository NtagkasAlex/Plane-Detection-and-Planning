import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from open3d.visualization.gui import MouseEvent, KeyEvent
from open3d.visualization.rendering import Camera
import numpy as np
import utility as U
import matplotlib.pyplot as plt
import conversions as C
import matplotlib.pyplot as plt
from Obstacles import Obstacles
import vis_graph as vg
import graph as g

def get_vis_graph(pcd_dbscan,pts_by_cluster,radius=0.1):
    #Get outline and Minkowski outline
    obstacles=Obstacles(pts_by_cluster)
    obstacles.get_outline()
    obstacles.minkowski()
    minkowski=obstacles.min
    

    polygons = C.list_to_graph(minkowski)
    #Building Graph
    workers=8
    print("Searching")
    graph = vg.VisGraph()
    print('Starting building visibility graph')
    graph.build(polygons, workers=workers)
    print('Finished building visibility graph')
    ref_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()

    return graph,obstacles
##Create Robot Cylinder
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=0.1)
    R = cylinder.get_rotation_matrix_from_xyz((-np.pi / 2,0,0))
    cylinder.rotate(R, center=(0, 0, 0))
    cylinder.translate(np.array([0,0,0]))

def clustering_objects(objects_pcd_file=None,objects_pcd=None,project=False):
    
    if objects_pcd is not None:
        pcd_dbscan = objects_pcd
    elif objects_pcd_file is not None:
        pcd_dbscan = o3d.io.read_point_cloud(objects_pcd_file)
    
    pointss=np.asarray(pcd_dbscan.points)
    if project:
        pointss=U.project_points_onto_plane(pointss,np.array([0,1,0]),-1)
    pcd_dbscan.points = o3d.utility.Vector3dVector(pointss)

    clusters = np.array(
            pcd_dbscan.cluster_dbscan(eps=0.2, min_points=10))
    num_clusters = len(set(clusters))
    
    colors = np.random.rand(num_clusters, 3)
    
    for i in range(len(clusters)):
        if clusters[i]==-1:
            clusters[i]=num_clusters-1
    
    colors[num_clusters-1,:]=np.zeros(3)

    print("Found "+str(num_clusters)+" objects")
    # Define colors for each cluster using a colormap
    point_colors = np.array([colors[label] for label in clusters])
    pcd_dbscan.colors = o3d.utility.Vector3dVector(point_colors)
    pts=np.asarray(pcd_dbscan.points)
    pts_proj=U.project_points_onto_plane(pts,np.array([0,1,0]),-1)
    pts_by_cluster=U.split_points_by_labels(pts_proj,clusters)
    return pcd_dbscan,pts_by_cluster