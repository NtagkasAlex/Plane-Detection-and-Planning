import numpy as np
from scipy.spatial import Delaunay

POINTS_PER_CIRCLE=10
def minkowski_sum_non_convex(non_convex_shape, circle_center, circle_radius):
    # Calculate Minkowski sum
    minkowski_points = []
    triangulation = Delaunay(non_convex_shape)
    for indexes in triangulation.simplices:
        for index in indexes:
            for angle in np.linspace(0, 2*np.pi, POINTS_PER_CIRCLE):
                x = non_convex_shape[index,0] + circle_radius * np.cos(angle) + circle_center[0]
                y = non_convex_shape[index,1] + circle_radius * np.sin(angle) + circle_center[1]
                minkowski_points.append([x, y])
        
   
    return np.array(minkowski_points)

def calculate_angle(v1, v2):
    angle= np.arctan2(v2[1] - v1[1], v2[0] - v1[0])
    # if angle<=0:
        # angle+=2*np.pi
    return angle


def minkowski_sum_test(P):
    R = []
    circle_radius = 0.1
    POINTS_PER_CIRCLE = 50  
    for angle in np.linspace(0, 2*np.pi, POINTS_PER_CIRCLE, endpoint=False):
        x = circle_radius * np.cos(angle) 
        y = circle_radius * np.sin(angle) 
        R.append([x, y])
    R = np.array(R)
    
    n = P.shape[0]
    m = R.shape[0]
    
    P = np.vstack((P, P[:2]))
    R = np.vstack((R, R[:2]))
    
    i = 0
    j = 0
    
    result = []

     
    while i < n-1 or j < m-1:
        result.append(P[i] + R[j])
        print(i,j)
        # if i == n:
        #     j += 1
        # elif j == m:
        #     i += 1
        # else:
        angle_P = calculate_angle(P[i], P[i + 1])
        angle_R = calculate_angle(R[j], R[j + 1])
        
        if angle_P > angle_R:
            i += 1
        elif angle_P < angle_R:
            j += 1
        else:
            i += 1
            j += 1
    
    return np.array(result)

def minkowski_sum(convex_shape, circle_center, circle_radius):
    # Calculate Minkowski sum
    minkowski_points = []
    for point in convex_shape:
        for angle in np.linspace(0, 2*np.pi, POINTS_PER_CIRCLE):
            x = point[0] + circle_radius * np.cos(angle) + circle_center[0]
            y = point[1] + circle_radius * np.sin(angle) + circle_center[1]
            minkowski_points.append([x, y])
    
    return np.array(minkowski_points)

