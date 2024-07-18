from collections import defaultdict
import numpy as np
from intervalTree import IntervalTree
class Point(object):

    def __init__(self, x, y, polygon_id=-1):
        self.x = float(x)
        self.y = float(y)
        self.polygon_id = polygon_id
        self.index=None
        self.internal=False
    def __eq__(self, point):
        return point and self.x == point.x and self.y == point.y

    def __ne__(self, point):
        return not self.__eq__(point)
    
    def __lt__(self, point):
        """ for heapq comparason"""
        return hash(self) < hash(point)
    
    def __hash__(self):
        return hash((self.x.__hash__() , self.y.__hash__()))


class Edge(object):

    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2

    def get_adjacent(self, point):
        if point == self.p1:
            return self.p2
        return self.p1

    def __contains__(self, point):
        return self.p1 == point or self.p2 == point

    def __eq__(self, edge):
        if self.p1 == edge.p1 and self.p2 == edge.p2:
            return True
        if self.p1 == edge.p2 and self.p2 == edge.p1:
            return True
        return False

    def __ne__(self, edge):
        return not self.__eq__(edge)


    def __hash__(self):
        return hash((self.p1.__hash__(), self.p2.__hash__()))


class Graph(object):


    def __init__(self, polygons):
        self.graph = defaultdict(set)
        self.edges = set()
        self.polygons = defaultdict(set)
        self.intervals=defaultdict(IntervalTree)
        self.poly_points=polygons
        id = 0
        for polygon in polygons:
            for i, point in enumerate(polygon):
                next_point = polygon[(i + 1) % len(polygon)]
                edge = Edge(point, next_point)
                point.polygon_id = id
                point.index=i
                next_point.index=(i+1)% len(polygon)
                next_point.polygon_id = id
                self.polygons[id].add(edge)
                self.add_edge(edge)
            id += 1

        for index in range(id):
            for edge in self.polygons[index]:
                self.intervals[index].put_edge(edge)


    def merge_polygons(self):
        id1,id2=self.ids
        poly1=self.polygons[id1]
        poly2=self.polygons[id2]
        from visible_vertices import edge_intersect,polygon_crossing,intersect_point
        point_indexes=[]
        for e1 in poly1:
            for e2 in poly2:
                if edge_intersect(e2.p1,e2.p2,e1):
                    point = intersect_point(e1.p1, e1.p2, e2)
                    if polygon_crossing(e1.p1,self,id2): 
                        if polygon_crossing(e2.p1,self,id1):
                            ##point,(in,out),(in,out)
                            point_indexes.append((point,(e1.p1.index,e1.p2.index),(e2.p1.index,e2.p2.index)))
                        else:
                            point_indexes.append((point,(e1.p1.index,e1.p2.index),(e2.p2.index,e2.p1.index)))
                    else: 
                        if polygon_crossing(e2.p1,self,id1):
                            ##point,(in,out),(in,out)
                            point_indexes.append((point,(e1.p2.index,e1.p1.index),(e2.p1.index,e2.p2.index)))
                        else:
                            point_indexes.append((point,(e1.p2.index,e1.p1.index),(e2.p2.index,e2.p1.index)))

        new_polygon=[]
        # for i in range(len(point_indexes)):
        # print(point_indexes)
        # print(len(self.poly_points))

        i=0
        # new_polygon.append(self.poly_points[id1][ptr])
        new_polygon.append(point_indexes[i][0])
        ptr1=point_indexes[i][1][1]##out index in poly2
        ptr1_stop=point_indexes[i+1][1][1]##out index in poly2
        ptr1_in=point_indexes[i][1][0]
        constant=1 if ptr1_in<=ptr1 else -1
        while not ptr1==ptr1_stop:
            new_polygon.append(self.poly_points[id1][ptr1])
            ptr1=ptr1+constant
            ptr1=ptr1%(len(self.poly_points[id1]))
        new_polygon.append(point_indexes[i+1][0])

        ptr2=point_indexes[i+1][2][1]
        ptr2_stop=point_indexes[i][2][1]##out index in poly2
        ptr2_in=point_indexes[i+1][2][0]
        constant=1 if ptr2_in<=ptr2 else -1

        while not ptr2==ptr2_stop:
            new_polygon.append(self.poly_points[id2][ptr2])
            ptr2=ptr2+constant
            ptr2=ptr2%(len(self.poly_points[id2]))       

        if id2>=id1:
            self.poly_points.pop(id2)
            self.poly_points.pop(id1)
        else:
            self.poly_points.pop(id1)
            self.poly_points.pop(id2)
        self.poly_points.append(new_polygon)

        # print(len(self.poly_points))

        return self.poly_points

        
    def has_intersecting_polygons(self):
        id = len(self.polygons)
        from visible_vertices import polygon_crossing
        for point in self.get_points():
            point_id=point.polygon_id
            for i in range(id):
                if i!=point_id:
                    if polygon_crossing(point,self,i):
                        print('inter')
                        self.ids=(i,point_id)
                        # print(self.ids)
                        point.internal=True
                        return True
        return False

        
    def get_adjacent_points(self, point):
        return [edge.get_adjacent(point) for edge in self[point]]

    def get_points(self):
        return list(self.graph.keys())

    def get_edges(self):
        return self.edges

    def add_edge(self, edge):
        self.graph[edge.p1].add(edge)
        self.graph[edge.p2].add(edge)
        self.edges.add(edge)
    def remove_internal(self):
        id = len(self.polygons)
        from visible_vertices import polygon_crossing
        for point in self.get_points():
        
            point_id=point.polygon_id
            for i in range(id):
                if i!=point_id:
                    if polygon_crossing(point,self,i):
                        print("yep")
                        point.internal=True
                        
                    
    def __contains__(self, item):
        if isinstance(item, Point):
            return item in self.graph
        if isinstance(item, Edge):
            return item in self.edges
        return False

    def __getitem__(self, point):
        if point in self.graph:
            return self.graph[point]
        return set()
