class IntervalTreeNode:
    def __init__(self, y1, y2, edge):
        self.y1 = y1
        self.y2 = y2
        self.max_y2 = y2
        self.edges = [edge]
        self.left = None
        self.right = None

    def __repr__(self):
        return f"Node(({self.y1}, {self.y2}), max_y2={self.max_y2}, edges={self.edges})"

class IntervalTree:
    def __init__(self):
        self.root = None

    def insert(self, root, y1, y2, edge):
        if not root:
            return IntervalTreeNode(y1, y2, edge)
        
        if y1 < root.y1:
            root.left = self.insert(root.left, y1, y2, edge)
        else:
            root.right = self.insert(root.right, y1, y2, edge)
        
        root.max_y2 = max(root.max_y2, y2)
        root.edges.append(edge)
        return root

    def put_edge(self, edge):
        y1, y2 = edge.p1.y, edge.p2.y
        y_max=max(y1,y2)
        y_min=min(y1,y2)
        y1=y_min
        y2=y_max
        if self.root is None:
            self.root = IntervalTreeNode(y1, y2, edge)
        else:
            self.root = self.insert(self.root, y1, y2, edge)

    def query(self, root, y):
    
        results = []
        if root.y1 <= y < root.y2:
            results.extend(root.edges)
        
        if root.left and root.left.max_y2 >= y:
            results.extend(self.query(root.left, y))
        
        if root.right and root.y1 < y:
            results.extend(self.query(root.right, y))
        
        return list(set(results))

    def get_edges(self, y):
        return self.query(self.root, y)
