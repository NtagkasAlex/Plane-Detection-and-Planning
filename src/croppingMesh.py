import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from open3d.visualization.gui import MouseEvent, KeyEvent
from open3d.visualization.rendering import Camera
import numpy as np
import utility as U
import copy
import os 
defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"

unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5


read_mesh = o3d.io.read_triangle_mesh("./area_3_mesh/semantic.obj")
pcd = o3d.geometry.PointCloud()

class AppWindow:

    def __init__(self, width, height, window_name="Lab"):

        self.w_width = width
        self.w_height = height
        self.first_click = True

        # initialize window & scene
        self.window = gui.Application.instance.create_window(window_name, width, height) 
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self._scene.scene.show_skybox(True)

        # basic layout
        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self._scene)

        self._scene.set_on_key(self._on_key_pressed)

        # set up camera
        self._set_camera()
        
        self.geometries = {}

    def _on_layout(self, layout_context):
        
        r = self.window.content_rect
        self._scene.frame = r

    def _set_camera(self):

        bounds = self._scene.scene.bounding_box
        center = bounds.get_center()
        self._scene.look_at(center, center - [0, 0, 5], [0, 1, 0])

    def add_geometry(self, geometry, name, shader = defaultUnlit, visible = True):
    
        self._scene.scene.add_geometry(name, geometry, shader)
        self.geometries[name] = geometry

        self._scene.scene.show_geometry(name, visible)

    def remove_geometry(self, name):

        self._scene.scene.remove_geometry(name)

    def _on_key_pressed(self, event):
        if event.key ==gui.KeyName.H:
            self._scene.scene.show_geometry("pcd", False)
        if event.key ==gui.KeyName.G:
            self._scene.scene.show_geometry("pcd", True)

        return gui.Widget.EventCallbackResult.IGNORED

if __name__=="__main__":

    gui.Application.instance.initialize()
    app = AppWindow(1024, 748)
    
    app.add_geometry(read_mesh,"mesh")
    lineset = o3d.geometry.LineSet.create_from_triangle_mesh(read_mesh)
    lineset.paint_uniform_color([0,0,0])
    app.add_geometry(lineset,"ls")
    mesh=copy.copy(read_mesh)

    ###load pcd###
    pcd_dir="./Area_3"
    points=np.array([0,0,0])
    colors=np.array([0,0,0])

   
    xyzrgb = np.loadtxt("./Area_3/hallway_1/hallway_1.txt", dtype=float) 
    point_rotated=xyzrgb[:,:3]@U.euler_angles_to_rotation_matrix([np.pi/2,0,0])+np.array([0,0,0])
    # points=np.vstack((points,xyzrgb[:,:3]))
    points=point_rotated
    colors=xyzrgb[:,3:6]/256

    
    pcd.points=o3d.utility.Vector3dVector(points)
    pcd.colors=o3d.utility.Vector3dVector(colors)
    
    
    app.add_geometry(pcd,"pcd")
    # new_mesh=o3d.geometry.crop_triangle_mesh()

    crop=True
    if crop:

        mesh=mesh.crop(pcd.get_axis_aligned_bounding_box())
        # pcd = new_mesh.sample_points_uniformly(4000000)
        app.add_geometry(mesh,"mesh")
        o3d.io.write_triangle_mesh("hallway.ply",mesh)
        ref_frame=U.create_ref_frame()
        app.add_geometry(ref_frame,"frame")
        # app.add_geometry(pcd,"pcd")
   
    gui.Application.instance.run()