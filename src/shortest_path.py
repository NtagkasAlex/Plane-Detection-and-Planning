import heapq
from visible_vertices import distance
INF=10000.
def dijkstra(graph, source,bounds=None):

    points = graph.get_points()
    
    dist = {vertex: INF for vertex in points}

    prev = {vertex: None for vertex in points}

    dist[source] = 0

    # store (distance, vertex) 
    priority_queue = [(0, source)]

    while priority_queue:
        #vertex in Q with minimum distance https://docs.python.org/2/library/heapq.html#basic-examples
        current_dist, u = heapq.heappop(priority_queue)

        if current_dist > dist[u]:
            continue

        # Explore neighbors of u
        for neighbor in graph.get_adjacent_points(u):

            if  bounds is not None and exclusion_criteria(neighbor,bounds):
                continue

            edge_length = distance(u, neighbor)
            alt = dist[u] + edge_length
            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = u
                heapq.heappush(priority_queue, (alt, neighbor))

    return dist, prev

def get_path(prev, destination):
    path = []
    while destination is not None:
        path.insert(0, destination)
        destination = prev[destination]
    return path

def exclusion_criteria(vertex,bounds):
    min=bounds[0]
    max=bounds[1]
    return vertex.x<=min[0] or vertex.x>=max[0] or vertex.y<=min[2] or vertex.y>=max[2]
