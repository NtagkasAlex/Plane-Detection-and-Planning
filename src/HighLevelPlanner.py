import numpy as np
import utility as U
import copy
import os 
import multiprocessing as mp
import DBSCAN as db
import DoorDetection as DD
import conversions as C
import Obstacles 
from sample import Sampler
from PathPlanning import get_vis_graph,clustering_objects
from room import Room
from collections import deque

THRESHHOLD=0.3
class highLevelPlanner:
    def __init__(self, rooms: list[Room]):
        self.rooms = rooms
        self.room_connections = {}  # To store room connections based on shared doors

    def build(self):
        self.roomNodes = []

        for i, room in enumerate(self.rooms):
            self.room_connections[room] = []
            for j, other_room in enumerate(self.rooms):
                if i != j:
                    for door1 in room.door_targets:
                        for door2 in other_room.door_targets:
                            if np.linalg.norm(door1 - door2) < THRESHHOLD:
                                self.room_connections[room].append(other_room)

    def short_path(self, start, end):
        for room in self.rooms:
            if start in room:
                start_room = room
            if end in room:
                end_room = room
                
        if start_room == end_room:
            return start_room.get_short_path(start, end)
        else:
            return self.multi_room_short_path(start_room, start, end_room, end)

    def multi_room_short_path(self, start_room, start, end_room, end):
        #Classic BFS approach for multi room navigation
        queue = deque([(start_room, start, [])])
        visited = set()

        while queue:
            current_room, current_point, path = queue.popleft()
            visited.add(current_room)

            if current_room == end_room:
                final_path = path + current_room.get_short_path(current_point, end)
                return final_path

            for next_room in self.room_connections[current_room]:
                if next_room not in visited:
                    for door1 in current_room.door_targets:
                        for door2 in next_room.door_targets:
                            if np.linalg.norm(door1 - door2) < THRESHHOLD:
                                average_point = (door1 + door2) / 2
                                intermediate_path = current_room.get_short_path(current_point, average_point)
                                queue.append((next_room, average_point, path + intermediate_path))
        
