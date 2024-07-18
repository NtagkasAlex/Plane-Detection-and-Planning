import numpy as np
import open3d as o3d
import Ransac as rc
import copy
import utility as U

EPS=0.05
MIN_POINTS=20
WEIGHTS=[2,1]
class DoorDetection:
    def __init__(self,min_bound,max_bound) -> None:
        self.max=max_bound
        self.min=min_bound
        self.gaps=[]
        self.inter_points=[]
        self._planes=None
    def projected_clustered_walls(self):
        proj_clustered_pcd=[]
        for i in range(len(self.outer_planes)):
            wall=self.outer_planes[i]
            points=np.asarray(wall.points)
            #Door Height Threshold 
            points=points[points[:,1]<(WEIGHTS[0]*self.max[1]+WEIGHTS[1]*self.min[1])/(WEIGHTS[0]+WEIGHTS[1])]
            points=points[points[:,1]>self.min[1]+EPS]

            #Project On XZ
            proj=o3d.geometry.PointCloud()
            projected_points=U.project_points_onto_plane(points,np.array([0,1,0]),-self.min[1])
            proj.points=o3d.utility.Vector3dVector(projected_points)
            clusters = np.array(proj.cluster_dbscan(eps=EPS, min_points=MIN_POINTS))

            num_clusters = len(set(clusters))
            pos_colors = np.random.rand(num_clusters, 3)
            colors=np.zeros(points.shape)
            for i in range(len(clusters)):
                colors[i]=pos_colors[clusters[i]]

                if clusters[i]==-1:
                    colors[i]=np.array([0,0,0])
            proj.colors = o3d.utility.Vector3dVector(colors)
            proj_clustered_pcd.append(proj)
        return proj_clustered_pcd

    @property
    def planes(self):
        return self._planes

    @planes.setter
    def planes(self, value):
        self._planes = value

    def clustering(self):
        
        outer_indexes=U.get_outer_planes(self._planes)
        self.outer_planes=[self.planes[i] for i in outer_indexes]
        self.debug=[]
        self.inter=[]
        for i in range(len(self.outer_planes)):

            wall=self.outer_planes[i]
            points=np.asarray(wall.points)
            # print(points)
            #Door Height Threshold 
            points=points[points[:,1]<(WEIGHTS[0]*self.max[1]+WEIGHTS[1]*self.min[1])/(WEIGHTS[0]+WEIGHTS[1])]
            points=points[points[:,1]>self.min[1]+EPS]
            #Project On XZ
            proj=o3d.geometry.PointCloud()
            projected_points=U.project_points_onto_plane(points,np.array([0,1,0]),-self.min[1])
            proj.points=o3d.utility.Vector3dVector(projected_points)
            clusters = np.array(proj.cluster_dbscan(eps=EPS, min_points=MIN_POINTS))
            

            self.debug.append(proj)
            ###
            dir=projected_points[0]-projected_points[1]
            angles=np.dot(projected_points,dir)
            indexes=np.argsort(angles)

            points=projected_points[indexes]
            clusters=clusters[indexes]
            change_points = np.where(np.diff(clusters) != 0)[0] + 1

            # List to store intervals as tuples (min_point, max_point, length)
            inter = []
            prev=None
            prev_cluster=None
            for i,cluster in enumerate(clusters):
                if cluster==-1:continue
                if prev is None or prev_cluster==cluster:
                    prev=points[i]
                    prev_cluster=cluster
                else:
                    inter.append((prev,points[i],np.linalg.norm(prev-points[i])))
                    prev=points[i]
                    prev_cluster=cluster
                    
            self.find_door(inter) 
        # print(self.inter_points)
        # print("gaps",self.gaps)

    def show_doorway(self):

        lines = np.array([[0, 1]])
        points=np.stack(self.inter_points[0])
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        colors = [[1, 0, 0]]  

        line_set.colors = o3d.utility.Vector3dVector(colors) 
        return line_set
        # o3d.visualization.draw_geometries(self.projected_clustered_walls()+[line_set])
    def show_doorways(self):
        points = []
        lines = []
        for i, point_pair in enumerate(self.doors):
            points.extend(point_pair)  
            lines.append([2 * i, 2 * i + 1]) 

        points = np.array(points)
        lines = np.array(lines)

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)

        colors = [[1, 0, 0] for _ in range(len(lines))]
        line_set.colors = o3d.utility.Vector3dVector(colors)

        return line_set

    def find_door(self,list_of_intervals):
        
        if len(list_of_intervals)==0:
            print("No door")
        else:

            for tup in list_of_intervals:
                p1,p2,gap=tup
                self.gaps.append(gap)
                self.inter_points.append([p1,p2])
    def get_doors(self):
        
        sorted_gaps=np.argsort(self.gaps)
        sorted_gaps = sorted_gaps[::-1]

        self.gaps= [self.gaps[i] for i in sorted_gaps]
        self.inter_points= [self.inter_points[i] for i in sorted_gaps]

    def get_doors_sized(self,door_length=0.88,eps=0.05):
        self.doors=[]
        for i in range(len(self.gaps)):
            if abs(self.gaps[i]-door_length)<eps:
                self.doors.append(self.inter_points[i])

        sorted_gaps=np.argsort(self.gaps)
        sorted_gaps = sorted_gaps[::-1]

        self.gaps= [self.gaps[i] for i in sorted_gaps]
        self.inter_points= [self.inter_points[i] for i in sorted_gaps]  

    
        
        

