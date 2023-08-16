import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import cv2

odometry_file_path = "./data/viewpoints_bessel_interpolation.tum"
pcd_file_path = "./data/Map.pcd"

with open(odometry_file_path, 'r') as f:
    content = f.readlines()
    position_lis = content

whole_pcd = o3d.io.read_point_cloud(pcd_file_path)

vis = o3d.visualization.Visualizer()
vis.create_window()
opt = vis.get_render_option()
opt.point_size = 1
opt.background_color = np.asarray([0, 0, 0])

vis.add_geometry(whole_pcd)
# OpenCV VideoWriter configuration
output_video_path = "./data/trajectory_video.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video_path, fourcc, 30, (1280, 720))


for idx in range(len(position_lis) - 1):
    origin_pos = [float(position_lis[idx].split(' ')[1]), float(position_lis[idx].split(' ')[2]), float(position_lis[idx].split(' ')[3]) + 3.0]
    ox, oy, oz, ow = [float(position_lis[idx].split(' ')[4]), float(position_lis[idx].split(' ')[5]), float(position_lis[idx].split(' ')[6]), float(position_lis[idx].split(' ')[7])]
    quaternion = np.array([ow, ox, oy, oz])
    rot_z_180 = np.array([[0, 0, 0, 1], [0, 0, -1, 0], [0, 1, 0, 0], [-1, 0, 0, 0]])  # x
    q_z = np.dot(rot_z_180, quaternion)
    R = o3d.geometry.get_rotation_matrix_from_quaternion(q_z)

    view_control = vis.get_view_control()
    front_vector = R[:, 0].tolist()
    up_vector = R[:, 2].tolist()

    view_control.set_front(front_vector)
    view_control.set_up(up_vector)
    view_control.set_lookat(origin_pos)
    view_control.set_zoom(0.001)

    vis.update_geometry(whole_pcd)
    vis.poll_events()
    vis.update_renderer()

       # Capture the current frame
    img = vis.capture_screen_float_buffer(False)
    img = (np.array(img) * 255).astype(np.uint8)

    # Resize the image to match the video dimensions
    img = cv2.resize(img, (1280, 720))

    # Write the frame to the video
    out.write(img)

out.release()
vis.destroy_window()




