from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)

import heapq
import numpy as np
import time
import matplotlib.pyplot as plt

class KdNode:
    def __init__(self, point, index, left_child, right_child):
        """
        Initializes a KdNode object with the given point, index, and child nodes.

        Args:
        - point: The point associated with the node.
        - index: The index of the point in the original points array.
        - left_child: The left child node.
        - right_child: The right child node.
        """
        self.point = point
        self.index = index
        self.left_child = left_child
        self.right_child = right_child

    @staticmethod
    def build_kd_node(pts: np.array) -> 'KdNode':
        """
        Build a k-d tree from a set of points.

        Args:
        - pts (np.array): The set of points.

        Returns:
        - KdNode: The root node of the constructed k-d tree.
        """
        def _build_kd_node(pts: np.array, indices: np.array, level: int) -> 'KdNode':
            """
            Recursively build a k-d tree from a set of points.

            Args:
            - pts (np.array): The set of points.
            - indices (np.array): The indices of the points in the original array.
            - level (int): The current level in the k-d tree.

            Returns:
            - KdNode: The root node of the constructed k-d tree.
            """
            if len(pts) == 0:
                return None
            else:
                dim = pts.shape[1]
                axis = level % dim

                # Sort points and indices based on the current axis.
                sorted_indices = np.argsort(pts[:, axis])
                sorted_pts = pts[sorted_indices]
                sorted_original_indices = indices[sorted_indices]

                # Find the median point.
                median_idx = len(sorted_indices) // 2
                split_point = sorted_pts[median_idx]
                split_index = sorted_original_indices[median_idx]

                # Recursively build left and right subtrees.
                pts_left = sorted_pts[:median_idx]
                indices_left = sorted_original_indices[:median_idx]
                pts_right = sorted_pts[median_idx + 1:]
                indices_right = sorted_original_indices[median_idx + 1:]
                
                left_child = _build_kd_node(pts_left, indices_left, level + 1)
                right_child = _build_kd_node(pts_right, indices_right, level + 1)
                return KdNode(split_point, split_index, left_child, right_child)

        # Initialize indices for the points
        indices = np.arange(pts.shape[0])
        # Start building the k-d tree from the root.
        root = _build_kd_node(pts, indices, 0)
        return root

    @staticmethod
    def getNodesBelow(root: 'KdNode') -> np.ndarray:
        """
        Static method to collect all points below a given node in the k-d tree.

        Args:
        - root: The root node from which to start collecting points.

        Returns:
        - An ndarray containing all points below the given node.
        """
        def _getNodesBelow(node: 'KdNode', pts):
            """
            Recursive function to traverse the k-d tree and collect points below the given node in a depth-first manner 

            Args:
            - node: The current node being visited.
            - pts: List to collect points
            """
            # Visit the left child first if it exists.
            if node.left_child:
                _getNodesBelow(node.left_child, pts)

            # Then visit the right child if it exists.
            if node.right_child:
                _getNodesBelow(node.right_child, pts)

            # Finally, append the point of the current node to the list.
            pts.append(node.point)

            return

        # Initialize an empty list to collect points.
        pts = []

        # Recursively collect points below the root.
        _getNodesBelow(root, pts)

        # Convert the list of points to a numpy array and return.
        return np.array(pts)

    @staticmethod
    def getNodesAtDepth(root: 'KdNode', depth: int) -> list:
        """
        Collects nodes at the specified depth in a k-d tree.

        Args:
        - root: The root node of the k-d tree.
        - depth: The depth at which to collect nodes.

        Returns:
        - List of nodes at the specified depth.
        """
        def _getNodesAtDepth(node: 'KdNode', nodes: list, depth: int) -> None:
            """
            Recursive function to traverse the k-d tree and collect nodes at the specified depth.

            Args:
            - node: The current node being visited.
            - nodes: List to collect nodes at the specified depth.
            - depth: The depth of the target nodes with respect the current node being visited 
            """
            # Base case: If depth is 0, append the current node to the nodes list.
            if depth == 0:
                nodes.append(node)
            else:
                # If depth is greater than 0, recursively traverse the left and right children.
                if node.left_child:
                    _getNodesAtDepth(node.left_child, nodes, depth - 1)

                if node.right_child:
                    _getNodesAtDepth(node.right_child, nodes, depth - 1)

            return

        # Initialize an empty list to collect nodes.
        nodes = []

        # Recursively collect nodes at the specified depth starting from the root node.
        _getNodesAtDepth(root, nodes, depth)

        # Return the list of nodes collected at the specified depth.
        return nodes
    


    @staticmethod
    def inSphere(_center,_radius, root: 'KdNode'):
        """
        Find points within a sphere in a k-d tree.

        Args:
        - sphere (Sphere3D): The sphere to search within.
        - root (KdNode): The root node of the k-d tree.

        Returns:
        - np.ndarray: An array of points within the specified sphere.
        """
        def _inSphere(root, center, radius, level, pts):
            """
            Recursively search for points within a sphere in a k-d tree.

            Args:
            - root (KdNode): The current node being visited.
            - center (tuple): The center coordinates of the sphere.
            - radius (float): The radius of the sphere.
            - level (int): The current level in the k-d tree.
            - pts (list): List to collect points within the sphere.

            Returns:
            - np.ndarray: An array of points within the specified sphere.
            """
            if root is None:
                return
            
            # Calculate the distance between the center of the sphere and the current node.
            dist = ((center - root.point)*(center - root.point)).sum()
            
            # Check if the current node is within the sphere.
            if dist <= radius**2:
                pts.append(root.index.item())
            
            # Determine which child node to visit first based on the current splitting axis.
            axis = level % 3
            if center[axis] - radius < root.point[axis]:
                if root.left_child:
                    _inSphere(root.left_child, center, radius, level + 1, pts)
            if center[axis] + radius >= root.point[axis]:
                if root.right_child:
                    _inSphere(root.right_child, center, radius, level + 1, pts)

            return pts
        
        

        # Initialize an empty list to collect points within the sphere.
        pts = []
        
        # Recursively search for points within the sphere starting from the root of the k-d tree.
        _inSphere(root, _center, _radius, 0, pts)

        # Convert the list of points to a NumPy array and return it.
        return pts
    @staticmethod
   

    @staticmethod
    def nearestK(test_pt: Point3D, root: 'KdNode', K: int) -> list['KdNode']:
        """
        Find the K nearest neighbors of a given test point in the k-d tree.

        Args:
        - test_pt (Point3D): The test point.
        - root (KdNode): The root of the k-d tree.
        - K (int): The number of nearest neighbors to find.

        Returns:
        - List[KdNode]: A list of K nearest neighbor nodes in the k-d tree.
        """

        def _nearestK(root: 'KdNode', test_pt, K, level, heap, dstar):
            """
            Recursively find the K nearest neighbors of the test point in the k-d tree.

            Args:
            - root (KdNode): The current node being visited.
            - test_pt (np.array): The coordinates of the test point.
            - K (int): The number of nearest neighbors to find.
            - level (int): The current level in the k-d tree.
            - heap (list): A min-heap to store the K nearest neighbors found so far.
            - dstar (float): The squared distance to the farthest neighbor in the heap.

            Returns:
            - float: The updated squared distance to the farthest neighbor in the heap.
            """
            if root is None:
                return dstar

            axis = level % 3  
            dist = ((test_pt - root.point)*(test_pt - root.point)).sum()
            
            if dist < dstar:
                heapq.heappush(heap, (-dist, root))
                if len(heap) > K:
                    _, _ = heapq.heappop(heap)
                dstar = -heap[0][0]

            d_ = test_pt[axis] - root.point[axis]
            is_on_left = d_ < 0 

            if is_on_left:
                if root.left_child: 
                    dstar= _nearestK(root.left_child, test_pt, K, level + 1, heap, dstar)
                if root.right_child and d_ ** 2 < dstar: 
                    dstar = _nearestK(root.right_child, test_pt, K, level + 1, heap, dstar)
            else:
                if root.right_child: 
                    dstar = _nearestK(root.right_child, test_pt, K, level + 1, heap, dstar)
                if root.left_child and d_ ** 2 < dstar:  
                    dstar = _nearestK(root.left_child, test_pt, K, level + 1, heap, dstar)

            return dstar

    

        # Convert the test point to a numpy array.
        test_pt = np.array([test_pt.x, test_pt.y, test_pt.z])

        # Initialize variables for nearest neighbor search.
        dstar = np.inf
        heap = []

        # Start the nearest neighbor search from the root of the k-d tree.
        _nearestK(root, test_pt, K, 0, heap, dstar)

        # Retrieve the K nearest neighbor nodes from the heap.
        nodes = []
        while heap:
            _, node = heapq.heappop(heap)
            nodes.append(node)
        
        return nodes    