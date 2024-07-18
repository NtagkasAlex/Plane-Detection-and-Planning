from __future__ import division
from math import pi, sqrt, atan, acos
from graph import Point
import numpy as np
INF = 10000.
CW = 1
CCW = -1
COLLINEAR = 0

def visible_vertices(point, graph, origin=None, destination=None):
    """Returns list of Points in graph visible by point.
    """
    edges = graph.get_edges()
    points = graph.get_points()
    if origin: points.append(origin)
    if destination: points.append(destination)
    points.sort(key=lambda p: (angle(point, p), distance(point, p)))
    
    # Initialize T with any intersecting edges on the half line from
    # point along the positive x-axis
    T = BalancedBST()
    point_inf = Point(INF, point.y)
    for edge in edges:
        if point in edge: continue
        if edge_intersect(point, point_inf, edge):
            if on_segment(point, edge.p1, point_inf): continue
            if on_segment(point, edge.p2, point_inf): continue
            T.insert(point, point_inf, edge)

    W = []
    prev = None
    prev_visible = None
    for p in points:
        # if p.internal:continue
        if p == point: continue

        
        is_visible=visible(point,p,graph,prev,prev_visible,T)
        if is_visible: W.append(p)

        # Update T - Add  clock wise edges incident on p
        for edge in graph[p]:
            if (point not in edge) and orientation(point, p, edge.get_adjacent(p)) == CW:
                T.insert(point, p, edge)
        # Update T - remove counter clock wise edges incident on p
        if T:
            for edge in graph[p]:
                if orientation(point, p, edge.get_adjacent(p)) == CCW:
                    T.delete(point, p, edge)
        
        prev = p
        prev_visible = is_visible
    return W

def visible(point,p,graph,prev,prev_visible,T):
    # Check if p is visible from point
    is_visible = False
    #  Check if the visible edge is interior to its polygon
    if  p not in graph.get_adjacent_points(point) and  edge_in_polygon(point, p, graph) :
        return is_visible
    # ...Non-collinear points
    if prev is None or orientation(point, prev, p) != COLLINEAR or not on_segment(point, prev, p):
        if len(T) == 0:
            is_visible = True
        elif not edge_intersect(point, p, T.left_most()):
            is_visible = True
        elif (edge_intersect(point, p, T.left_most()) and p in T.left_most() ) or \
              (edge_intersect(point, p, T.left_most()) and p in T.left_most()  and orientation(point,p,T.left_most().get_adjacent(p))):
            is_visible=True

    # ...For collinear points, if previous point was not visible, p is not
    elif not prev_visible:
        is_visible = False
    # ...For collinear points, if previous point was visible, need to check
    # that the edge from prev to p does not intersect any open edge.
    else:
        is_visible = True
        for edge in T:
            if prev not in edge and edge_intersect(prev, p, edge):
                is_visible = False
                break
    
    return is_visible

def edge_in_polygon(p1, p2, graph):
    """Return true if the edge from p1 to p2 is interior to any polygon
    in graph."""
    if p1.polygon_id != p2.polygon_id:
        return False
    if p1.polygon_id == -1 or p2.polygon_id == -1:
        return False
    mid_point = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
    
    return polygon_crossing(mid_point,graph,p1.polygon_id)

def polygon_crossing(p1,graph,id):
    """Returns True if Point p1 is internal to the polygon with id """
    p2 = Point(INF, p1.y)
    intersect_count = 0
    edges=graph.intervals[id].get_edges(p1.y)
    # for edge in edges:
        # print(edge.p1.y,edge.p2.y)
    for edge in edges :
        if p1.x > edge.p1.x and p1.x > edge.p2.x: continue
        # Deal with points collinear to p1
        edge_p1_collinear = (orientation(p1, edge.p1, p2) == COLLINEAR)
        edge_p2_collinear = (orientation(p1, edge.p2, p2) == COLLINEAR)
        if edge_p1_collinear and edge_p2_collinear: continue
        if edge_p1_collinear or edge_p2_collinear:
            collinear_point = edge.p1 if edge_p1_collinear else edge.p2
            if edge.get_adjacent(collinear_point).y > p1.y:
                intersect_count += 1
                
        elif edge_intersect(p1, p2, edge):
            intersect_count += 1
    if intersect_count % 2 == 0:
        return False
    return True



def distance(p1, p2):
    return np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)


def intersect_point(p1, p2, edge):
    """Return Point of inter of p1p2 and edge"""
    if p1 in edge: return p1
    if p2 in edge: return p2
    if edge.p1.x == edge.p2.x:
        if p1.x == p2.x:
            return None
        pslope = (p1.y - p2.y) / (p1.x - p2.x)
        intersect_x = edge.p1.x
        intersect_y = pslope * (intersect_x - p1.x) + p1.y
        return Point(intersect_x, intersect_y)

    if p1.x == p2.x:
        eslope = (edge.p1.y - edge.p2.y) / (edge.p1.x - edge.p2.x)
        intersect_x = p1.x
        intersect_y = eslope * (intersect_x - edge.p1.x) + edge.p1.y
        return Point(intersect_x, intersect_y)

    pslope = (p1.y - p2.y) / (p1.x - p2.x)
    eslope = (edge.p1.y - edge.p2.y) / (edge.p1.x - edge.p2.x)
    if eslope == pslope:
        return None
    intersect_x = (eslope * edge.p1.x - pslope * p1.x + p1.y - edge.p1.y) / (eslope - pslope)
    intersect_y = eslope * (intersect_x - edge.p1.x) + edge.p1.y
    return Point(intersect_x, intersect_y)


def angle(center, point):
    dx = point.x - center.x
    dy = point.y - center.y
    angle=np.arctan2(dy,dx)
    if angle <0:
        angle+=2*np.pi
    
    return angle


def cosine_theorem(point_a, point_b, point_c):
    """returns angle between b and c
    """
    a = (point_c.x - point_b.x)**2 + (point_c.y - point_b.y)**2
    b = (point_c.x - point_a.x)**2 + (point_c.y - point_a.y)**2
    c = (point_b.x - point_a.x)**2 + (point_b.y - point_a.y)**2
    if a==0 or c==0:
        return -INF
    cos_value = (a + c - b) / (2 * sqrt(a) * sqrt(c))
    if abs(cos_value)>1:
        return INF
    return acos(cos_value)


def orientation(A, B, C):
    """Return 1 if counter clockwise, -1 if clock wise, 0 if collinear """
    orientation = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x)
    
    if orientation > 0: return 1
    if orientation < 0: return -1
    return 0


def on_segment(p, q, r):
    """Given three colinear points p, q, r-> True if point q
    lies on pr"""
    if (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)):
        if (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y)):
            return True
    return False


def edge_intersect_interior(p1, q1, edge):
    """Same as below but only internaly """
    p2 = edge.p1
    q2 = edge.p2
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if (o1 != o2 and o3 != o4):
        return True

    return False

def edge_intersect(p1, q1, edge):
    """Return True if p1q1 and edge intesect
    http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/"""
    p2 = edge.p1
    q2 = edge.p2
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if (o1 != o2 and o3 != o4):
        return True
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if o1 == COLLINEAR and on_segment(p1, p2, q1):
        return True
    # p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if o2 == COLLINEAR and on_segment(p1, q2, q1):
        return True
    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if o3 == COLLINEAR and on_segment(p2, p1, q2):
        return True
    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if o4 == COLLINEAR and on_segment(p2, q1, q2):
        return True
    return False


class BalancedBST(object):
    def __init__(self):
        self.edges = []

    def insert(self, p1, p2, edge):
        self.edges.insert(self.index(p1, p2, edge), edge)

    def delete(self, p1, p2, edge):
        index = self.index(p1, p2, edge) - 1
        if self.edges[index] == edge:
            del self.edges[index]

    def left_most(self):
        return self.edges[0]

    def less_than(self, p1, p2, edge1, edge2):
        """Return True if edge1 is smaller than edge2, False otherwise."""
        if edge1 == edge2:
            return False
        if not edge_intersect(p1, p2, edge2):
            return True
        edge1_dist=distance(p1,intersect_point(p1, p2, edge1))
        edge2_dist=distance(p1,intersect_point(p1, p2, edge2))

        if edge1_dist > edge2_dist:
            return False
        if edge1_dist < edge2_dist:
            return True
        # If the distance is equal, we need to compare on the edge angles.
        if edge1_dist == edge2_dist:
            if edge1.p1 in edge2:
                same_point = edge1.p1
            else:
                same_point = edge1.p2
            angle_edge1 = cosine_theorem(p1, p2, edge1.get_adjacent(same_point))
            angle_edge2 = cosine_theorem(p1, p2, edge2.get_adjacent(same_point))
            if angle_edge1 < angle_edge2:
                return True
            return False

    def index(self, p1, p2, edge):
        lo = 0
        hi = len(self.edges)
        while lo < hi:
            mid = (lo+hi)//2
            if self.less_than(p1, p2, edge, self.edges[mid]):
                hi = mid
            else:
                lo = mid + 1
        return lo

    def __len__(self):
        return len(self.edges)

    def __getitem__(self, index):
        return self.edges[index]