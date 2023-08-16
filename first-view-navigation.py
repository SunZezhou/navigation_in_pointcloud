import os
os.environ['NUMEXPR_MAX_THREADS'] = '16'
os.environ['NUMEXPR_NUM_THREADS'] = '8'
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R

odometry_file_path = "./data/viewpoints_bessel_interpolation.tum"
pcd_file_path = "./data/Map.pcd"

with open(odometry_file_path, 'r') as f:
    content = f.readlines()
    position_lis = content

whole_pcd = o3d.io.read_point_cloud(pcd_file_path)

vis=o3d.visualization.Visualizer()
vis.create_window()
opt = vis.get_render_option()
opt.point_size = 1
opt.background_color = np.asarray([0,0,0])

whole_pcd.paint_uniform_color([1,1,1])  # point color all white
vis.add_geometry(whole_pcd)
# vis.run()

# odometry of mine format is: timestamp,  pos_x, pos_y, pos_z, ox, oy, oz, ow
for idx in range(len(position_lis) - 1):
    origin_pos = [float(position_lis[idx].split(' ')[1]), float(position_lis[idx].split(' ')[2]), float(position_lis[idx].split(' ')[3])+3.0]
    ox, oy, oz, ow = [float(position_lis[idx].split(' ')[4]), float(position_lis[idx].split(' ')[5]), float(position_lis[idx].split(' ')[6]), float(position_lis[idx].split(' ')[7])]
    quaternion = np.array([ow, ox, oy, oz])
    rot_z_180 = np.array([[0, 0, 0, 1], [0, 0, -1, 0], [0, 1, 0, 0], [-1, 0, 0, 0]])  # x
    q_z = np.dot(rot_z_180, quaternion)
    R = o3d.geometry.get_rotation_matrix_from_quaternion(q_z)

    view_control = vis.get_view_control()
    front_vector = R[:, 0].tolist()
    up_vector = R[:, 2].tolist()
    # set up/lookat/front vector to vis
    view_control.set_front(front_vector)    # Vector pointing vertically off-screen
    view_control.set_up(up_vector)          # Vector pointing vertically top of the screen
    view_control.set_lookat(origin_pos)
    # because I want get first person view, so set zoom value with 0.001, if set 0, there will be nothing on the screen
    view_control.set_zoom(0.001)
    vis.update_geometry(whole_pcd)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()

