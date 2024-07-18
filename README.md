# Plane Detection and Path Planning

## Overview
This project aims to develop a robust pipeline for navigating and path planning through dense indoor environments using the Stanford Building Parser Dataset. The dataset is based on realistic measurements obtained via LiDAR and depth cameras. The general framework for achieving this includes:

1. Detecting all walls of the room.
2. Clustering different objects.
3. Detecting the door.
4. Performing path planning and avoiding obstacles.

## Plane Detection - 3D Mesh
RANSAC (Random Sample Consensus) is used for plane detection in 3D point clouds by randomly selecting subsets of points to fit a plane model and identifying inliers that conform to the model within a specified tolerance. The implementation includes:
- Applying RANSAC on a 3D mesh using the number of vertices and the area as metrics.
- Iteratively implementing RANSAC and removing inliers in each step.
- Post-processing to classify planes and objects based on their orientation and position.

## Plane Detection - Point Cloud
To sample the mesh and create a Point Cloud:
- Uniform sampling of triangles using density and area.
- Implementing RANSAC algorithm on the Point Cloud.
- Post-processing to remove horizontal planes that are not the ceiling or floor, and resolving object points belonging to planes.

## Object Clustering
DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is used for object segmentation in Point Clouds. The implementation involves:
- Querying points within a radius using KDTree for efficiency.
- Clustering object points and visualizing different cluster results.

## Door Detection
To detect doors:
1. Project wall points on the floor.
2. Sort points on a 3D line.
3. Use Convex Hull Graham Scan to find outside walls.
4. Project, cluster, and sort wall points to detect door-sized gaps.

## Path Planning
The path planning involves:
- Using visibility graphs for optimal path finding.
- Preprocessing obstacles using alpha-shapes and Minkowski sum.
- Implementing Interval Tree for efficient edge intersection queries.
- Merging intersecting polygons to avoid issues in path planning.

### Single Room Path Planning
Dijkstra's algorithm is used for finding the shortest path between two points in a single room. The visibility graph is constructed, and Dijkstra's algorithm is applied to find the shortest path.

### Multiple Room Path Planning
For multiple rooms:
1. Calculate visibility graphs for each room.
2. Calculate connections between rooms using door coordinates.
3. Use BFS to find the shortest path through the room connections.

## Project Structure
- `src` folder: Contains all the source code.
- `roomX` folders: Contains a `Planes` subfolder with all the plane Point Clouds, a `mesh.ply` file with the room mesh, and an `objects.ply` file with the objects.

## Running the Project
### Single Room Example
To run the single room example:
```bash
python singleRoom.py roomname
```
Replace `roomname` with the desired room (e.g., room1, room2).
### Multiple Room Example
To run the multiple room example:
```bash
python multiRoom.py
```

![texxt](images/image%20copy%2010.png)
