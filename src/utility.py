import open3d as o3d
import numpy as np
import copy
import os
from math import sqrt
import shutil

import multiprocessing
from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)

def delete_contents(directory):
    # Check if the directory exists
    if not os.path.exists(directory):
        print(f"The directory {directory} does not exist.")
        return

    # Iterate over the items in the directory
    for item in os.listdir(directory):
        item_path = os.path.join(directory, item)
        
        # Skip the mesh.ply file
        if item == 'mesh.ply':
            continue
        
        # Check if the item is a file or a directory
        if os.path.isfile(item_path) or os.path.islink(item_path):
            os.unlink(item_path)  # Remove file or link
        elif os.path.isdir(item_path):
            # Iterate over the items in the subdirectory and remove them
            for subitem in os.listdir(item_path):
                subitem_path = os.path.join(item_path, subitem)
                if os.path.isfile(subitem_path) or os.path.islink(subitem_path):
                    os.unlink(subitem_path)  # Remove file or link
                elif os.path.isdir(subitem_path):
                    shutil.rmtree(subitem_path)  # Remove subdirectory and all its contents

    print(f"Deletion Complete... Saving new Files")
def project_points_onto_plane(points, plane_normal, d):
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    
    dist = np.dot(points, plane_normal) + d
    dist = dist.reshape(-1,1)
    
    projected_points = points - dist * plane_normal
    
    
    return np.array(projected_points)
def vector_cos(v1,v2):
    res=np.dot(v1,v2)
    #####MAYBE TURN IT BACK###
    if v1.shape[0]==3:
        res/=np.linalg.norm(v1)*np.linalg.norm(v2)

    else:
        res/=np.linalg.norm(v1,axis=1)*np.linalg.norm(v2)
    return abs(res)
def load_planes(path):
    n_planes=0
    planes=[]
    for area_path in os.listdir(path):
        n_planes+=1
    ind=0
    for area_path in os.listdir(path):
        plane = o3d.io.read_point_cloud(path+"/planes"+str(ind)+".ply")
        planes.append(plane)
        ind+=1
    return planes
def get_plane_equations(planes):
    n_planes=len(planes)
    plane_eq=[0]*n_planes

    for i in range(n_planes):
        pcd=planes[i]
        points=np.asarray(pcd.points)
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        u, s, vh = np.linalg.svd(centered_points)
        normal = vh[2, :]    
        d = -np.dot(normal, centroid)
        plane_eq[i]=(normal[0], normal[1], normal[2], d)
    return plane_eq
def get_line_interval(points):
    v= points[1]-points[2]
    # v=np.array([1,0,0])
    angles=np.dot(points,v)

    # angles=np.linalg.norm()
    min_angle_ind=np.argmin(angles,axis=0)
    max_angle_ind=np.argmax(angles,axis=0)

    return min_angle_ind,max_angle_ind
def get_line_midpoint(plane):
    '''
    Plane in pcd form
    Returns:
    mid point in XZ coordinates
    '''
    plane_points=np.asarray(plane.points)
    pcd=plane
    points=np.asarray(pcd.points)
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    u, s, vh = np.linalg.svd(centered_points)
    normal = vh[2, :]    
    d = -np.dot(normal, centroid)
    points=project_points_onto_plane(plane_points,np.array([0,1,0]),0)
    axis=np.array([1,0,0])
    if np.dot(normal,axis)>0.9:
        axis=np.array([0,0,1])

    angles=np.dot(points,axis)

    # angles=np.linalg.norm()
    min_angle_ind=np.argmin(angles,axis=0)
    max_angle_ind=np.argmax(angles,axis=0)

    center_point=(points[min_angle_ind]+points[max_angle_ind])/2
    return center_point

def triangle_area(v1,v2,v3):

    return 1/2*np.linalg.norm(np.cross(v2-v1,v3-v1))
# Contruct a default plane pointing in the upward y direction 

def get_plane(vertices):

    # Define faces of the plane (two triangles)
    faces = np.array([[0, 1, 2],  # Triangle 1 (vertices 0-1-2)
                      [0, 2, 3]]) # Triangle 2 (vertices 0-2-3)

    # Create Open3D TriangleMesh object
    plane_mesh = Mesh3D(color=Color.GRAY)
    plane_mesh.vertices = vertices
    plane_mesh.triangles = faces
    return plane_mesh
def get_floor_plane(min,planes):
    outer_planes=[planes[i] for i in get_outer_planes(planes)]
    eq=get_plane_equations(outer_planes)
    line_eq=[(x[0],x[2],x[3]) for x in eq]
    points=[]
    for i in range(len(outer_planes)):
        line1=line_eq[i]
        line2=line_eq[(i+1)%len(outer_planes)]
        inter=find_intersection(line1,line2)
        # print(inter)
        points.append(inter)
        # points.append(convert_2d_to_3d(np.array(inter).reshape(1,2),planes[0].get_min_bound()[1]))
    return convert_2d_to_3d(np.array(points),min)
def find_intersection(line1, line2):
    """
    Find the intersection of two lines given in the form [a, b, c] where the equation is ax + by = c.
    
    Parameters:
    line1 (list): Coefficients [a1, b1, c1] for the first line.
    line2 (list): Coefficients [a2, b2, c2] for the second line.
    
    Returns:
    tuple: (x, y) coordinates of the intersection point or None if lines are parallel or coincident.
    """
    a1, b1, c1 = line1
    a2, b2, c2 = line2
    
    # Calculate the determinant
    det = a1 * b2 - a2 * b1
    
    if det == 0:
        # Lines are parallel or coincident
        print("aaa")
        return None
    
    # Using Cramer's rule to find the intersection point
    x = (c1 * b2 - c2 * b1) / det
    y = (a1 * c2 - a2 * c1) / det
    
    return [x, y]


def transform(data, euler_angles, translation):
    """
    Transform a mesh or point cloud by applying translation and rotation.

    Parameters:
    - data: Open3D TriangleMesh or PointCloud object
    - translation: Translation vector [tx, ty, tz]
    - euler_angles: Euler angles [rx, ry, rz] in radians

    Returns:
    - Transformed mesh or point cloud
    """
    # Get vertices or points
    if isinstance(data, o3d.geometry.TriangleMesh):
        vertices = np.asarray(data.vertices)
    elif isinstance(data, o3d.geometry.PointCloud):
        vertices = np.asarray(data.points)

    # Convert Euler angles to rotation matrix
    rotation_matrix = euler_angles_to_rotation_matrix(euler_angles)

    # Apply rigid transformation
    transformed_vertices = np.dot(vertices, rotation_matrix.T) + translation 

    # Update vertices or points
    if isinstance(data, o3d.geometry.TriangleMesh):
        data.vertices = o3d.utility.Vector3dVector(transformed_vertices)
    elif isinstance(data, o3d.geometry.PointCloud):
        data.points = o3d.utility.Vector3dVector(transformed_vertices)

    return

# Generate a plane with given position and orientation  
    

def euler_angles_to_rotation_matrix(euler_angles):
    """
    Create a rotation matrix from Euler angles.

    Parameters:
    - euler_angles: Euler angles [rx, ry, rz] in radians

    Returns:
    - Rotation matrix
    """
    rx, ry, rz = euler_angles

    rotation_matrix_x = np.array([[1, 0, 0],
                                   [0, np.cos(rx), -np.sin(rx)],
                                   [0, np.sin(rx), np.cos(rx)]])

    rotation_matrix_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                                   [0, 1, 0],
                                   [-np.sin(ry), 0, np.cos(ry)]])

    rotation_matrix_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                                   [np.sin(rz), np.cos(rz), 0],
                                   [0, 0, 1]])
    # Combine rotation matrices
    rotation_matrix = rotation_matrix_z @ rotation_matrix_y @ rotation_matrix_x
    
    return rotation_matrix

# Determine the positions of points relative to a plane defined by its parameters.
def split_points_by_labels(points, labels):
    # Initialize a dictionary to store points for each label
    label_to_points = {label: [] for label in set(labels)}

    # Iterate over labels and points and append points to the corresponding label list
    for label, point in zip(labels, points):
        label_to_points[label].append(point)

    # Convert lists to NumPy arrays for each label
    for label in label_to_points:
        label_to_points[label] = np.array(label_to_points[label])

    return label_to_points

def convert_2d_to_3d(points,y=0):
    out_points=np.zeros(shape=(points.shape[0],3))
    out_points[:,0]=points[:,0]
    out_points[:,1]=y
    out_points[:,2]=points[:,1]
    return out_points

def create_poly_3d(points,color=[0,0,0]):
    if points.shape[1]==2:
        points=convert_2d_to_3d(points)
    edges = []
    num_points = len(points)
    for i in range(num_points):
        edges.append([i, (i + 1) % num_points])  # Connect each point to the next, and connect the last point to the first

    # Create a LineSet from the edges
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(edges)
    line_set.paint_uniform_color(color)
    return line_set

def center(data):
    if isinstance(data, o3d.geometry.TriangleMesh):
        vertices = np.asarray(data.vertices)
    elif isinstance(data, o3d.geometry.PointCloud):
        vertices = np.asarray(data.points)
    vertices-=np.mean(vertices,axis=0)
    if isinstance(data, o3d.geometry.TriangleMesh):
        data.vertices = o3d.utility.Vector3dVector(vertices)
    elif isinstance(data, o3d.geometry.PointCloud):
        data.points = o3d.utility.Vector3dVector(vertices)
    return


def get_outer_planes(planes:list)->list:
    '''Gets a list of planes (horizontal or not) and returns the indexes to outer planes'''
    mid_points=[]
    mid_point_idx=[]
    n_planes=len(planes)
    plane_eq=get_plane_equations(planes)
    for i in range(n_planes):
        normal=np.array(plane_eq[i][:3])
        if abs(vector_cos(normal,np.array([0,1,0])))<0.9:
            mid_points.append(get_line_midpoint(planes[i]))
            mid_point_idx.append(i)
    points=np.array(mid_points)
    points=np.delete(points,1,1)
    outer_planes_idx=CH_graham_scan(points)#indexes from planes vertical planes
    indexes=[mid_point_idx[i] for i in outer_planes_idx]#general indexes in all planes
    # return mid_point_idx
    return indexes

def CH_graham_scan(points):
    
        def orientation(p, q, r):
            v1=q-p
            v2=r-q
            return np.cross(v1,v2) 

        #initial step
        # sorting according to angle
        # points=np.array(points_list)
        # points=np.delete(points,1,1)
        p0_idx = np.argmin(points[:,1])
        p0 = points[p0_idx]
        # Calculate the polar angle of each point with respect to p0
        angles = np.arctan2((points[:,1]-p0[1]),(points[:,0]-p0[0]))
        # Sort the points by angle
        sorted_idx = np.argsort(angles)
        sorted_points = points[sorted_idx]

        stack = [0, 1]
        ids = np.arange(2, sorted_points.shape[0])
        #iterating over the rest of the points
        for id in ids:
            # pop elements from the stack until graham condition is satisfied
            while len(stack) > 1 and not (orientation(sorted_points[stack[-2]], sorted_points[stack[-1]], sorted_points[id])) > 0:                
                stack.pop()
            #append current point
            stack.append(id)

        # points=sorted_points[stack]
        return  sorted_idx[stack]  

def lineset_from_edges(edges:list):
    points=[]
    for edge in edges:
        points.append(edge[0])
        points.append(edge[1])
    points=np.array(points)
    points=convert_2d_to_3d(points)
    edges = []
    num_points = points.shape[0]
    for i in range(num_points-1):
        edges.append([i, (i + 1)])  # Connect each point to the next, and connect the last point to the first

    # Create a LineSet from the edges
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(edges)
    line_set.paint_uniform_color([0,0,0])
    return line_set
def lineset_for_vertex(edge_list:list,reduced=False):
    linesets=[]
    for i,edges in enumerate(edge_list):            
        if not(reduced and not i%15==0):continue
        points= np.vstack([np.vstack(pair) for pair in edges])
        edges = []
        num_points = points.shape[0]
        for i in range(num_points-1):
            edges.append([i, (i + 1)]) 
        line_set = o3d.geometry.LineSet()
        # print(points.shape)
        line_set.points = o3d.utility.Vector3dVector(convert_2d_to_3d(points))
        line_set.lines = o3d.utility.Vector2iVector(edges)
        line_set.paint_uniform_color([0,0,0])
        linesets.append(line_set)
    return linesets
def create_point_cloud_outlines(outlines):

    pcds=[]
    for outline in outlines:
        pcd=o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(convert_2d_to_3d(outline))
        pcd.paint_uniform_color(np.random.rand(3))
        pcds.append(pcd)
    return pcds
def show_path(points):
    out_points=np.zeros(shape=(points.shape[0],3))
    out_points[:,0]=points[:,0]
    out_points[:,2]=points[:,1]
    edges = []
    num_points = len(out_points)
    for i in range(num_points-1):
        edges.append([i, (i + 1)])  # Connect each point to the next, and connect the last point to the first

    # Create a LineSet from the edges
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(out_points)
    line_set.lines = o3d.utility.Vector2iVector(edges)
    line_set.paint_uniform_color([0,1,0])
    return line_set