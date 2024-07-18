import numpy as np
from scipy.spatial import Delaunay
import math
import utility as U

def calculate_circumcircle_radius(p1,p2,p3):
    a = np.linalg.norm(p2-p3)
    b = np.linalg.norm(p1-p3)
    c = np.linalg.norm(p2-p1)
    
    # area = calculate_area(x1, y1, x2, y2, x3, y3)
    area=U.triangle_area(p1,p2,p3)
    
    if area == 0:
        area=0.00001
    
    radius = (a * b * c) / (4 * area)
    return radius

def remove_duplicates(edges):
    edge_list = [tuple(sorted(edge)) for edge in edges]
    edge_count = {}
    for edge in edge_list:
        if edge in edge_count:
            edge_count[edge] += 1
        else:
            edge_count[edge] = 1

    unique_edges = [edges[i] for i, edge in enumerate(edge_list) if edge_count[edge] == 1]
    return unique_edges



def order_points(edges, points):

    neighbors = {}
    for i1, i2 in edges:
        if i1 not in neighbors:
            neighbors[i1] = []
        if i2 not in neighbors:
            neighbors[i2] = []
        neighbors[i1].append(i2)
        neighbors[i2].append(i1)

    # Find the starting point, any point can be a start
    start = edges[0][0]
    ordered_indices = [start]
    
    current = neighbors[start][0]
    ordered_indices.append(current)
    
    for _ in range(len(points) - 2):
        next_point = neighbors[current][0] if neighbors[current][0] != ordered_indices[-2] else neighbors[current][1]
        if next_point in ordered_indices:
            continue
        ordered_indices.append(next_point)
        current = next_point

    ordered_points = [points[i] for i in ordered_indices]

    return ordered_points
def get_outline(points, alpha: float):
    
    edges=[]    
    
    delauney_triangles = Delaunay(points)
    for ind_vertices in delauney_triangles.simplices:

        ind1, ind2, ind3 = ind_vertices

        p1=points[ind1]
        p2=points[ind2]
        p3=points[ind3]
        radius=calculate_circumcircle_radius(p1,p2,p3)
        if radius <= alpha:
            
            edges.append((ind1,ind2))
            edges.append((ind2,ind3))
            edges.append((ind1,ind3))
    
    edges=remove_duplicates(edges)
    points=order_points(edges,points)

    return points