from multiprocessing import Pool
import numpy as np
from graph import Graph, Edge
from shortest_path import dijkstra,get_path
from visible_vertices import visible_vertices
import copy
class VisGraph(object):

    def __init__(self):
        self.graph = None
        self.visgraph = None


    def build(self, list_of_obstacles, workers=1): 
       

        self.graph = Graph(list_of_obstacles)
        self.visgraph = Graph([])
        while self.graph.has_intersecting_polygons():
            polygons=self.graph.merge_polygons()
            self.graph = Graph(polygons)
        points = self.graph.get_points()
        batch_size = 30

        
        if workers==1:
            for batch in [points[i:i + batch_size]
                    for i in range(0, len(points), batch_size)]:
                for edge in find_edges(self.graph, batch):
                    self.visgraph.add_edge(edge)
        else:
            pool = Pool(workers)
            batches = [(self.graph, points[i:i + batch_size])
                        for i in range(0, len(points), batch_size)]

            results = list(pool.starmap(find_edges, batches))

            for result in results:
                for edge in result:
                    self.visgraph.add_edge(edge)
    def get_poly(self):
        my_poly=[]
        for poly in self.graph.poly_points:
            polyy=[]
            for point in poly:
                polyy.append(np.array([point.x,point.y]))
            my_poly.append(polyy)
        return my_poly
    def show(self):
        edges=[]
        for point in self.visgraph.get_points():
            edge=[]
            adj_points=self.visgraph.get_adjacent_points(point)
            for point2 in adj_points:
                edge.append((np.array([point.x,point.y]),np.array([point2.x,point2.y])))
            edges.append(edge)
        return edges


    def find_visible(self, point):
        return visible_vertices(point, self.graph)

    def update(self, points, origin=None, destination=None):
        for p in points:
            for v in visible_vertices(p, self.graph, origin=origin,
                                      destination=destination):
                self.visgraph.add_edge(Edge(p, v))

    def shortest_path(self, origin, destination,bounds=None):
        """Find and return shortest path between origin and destination.
        """

        dijstra_graph=copy.copy(self.visgraph)
        
        for v in visible_vertices(origin, self.graph, destination=destination):
            dijstra_graph.add_edge(Edge(origin, v))
        for v in visible_vertices(destination, self.graph, origin=origin):
            dijstra_graph.add_edge(Edge(destination, v))

        dist, prev = dijkstra(dijstra_graph ,origin,bounds)
        path = get_path(prev, destination)
        return path
        
    

def find_edges(graph, points):
    visible_edges = []

    for p1 in points:
        for p2 in visible_vertices(p1, graph):
            visible_edges.append(Edge(p1, p2))
    return visible_edges
