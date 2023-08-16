import os
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# 设置文件路径和保存路径
input_path = "./data/viewpoints.txt"
output_path = "./data/viewpoints_bessel_interpolation.tum"

# 读取文件内容
with open(input_path, 'r') as f:
    lines = f.readlines()

positions = []
quaternions = []

# 解析文件内容
for line in lines:
    parts = line.split()
    positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    quaternions.append([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])

positions = np.array(positions)
quaternions = np.array(quaternions)

# 过滤掉四元数全为0的数据
valid_indices = np.where(np.linalg.norm(quaternions, axis=1) > 0)
positions = positions[valid_indices]
quaternions = quaternions[valid_indices]

# 拟合贝塞尔曲线
t = np.linspace(0, 1, len(positions))
position_splines = [interp1d(t, pos) for pos in positions.T]
quaternion_splines = [interp1d(t, quat, kind='quadratic') for quat in quaternions.T]

# 生成新的时间戳并计算平滑后的位置和四元数
new_t = np.linspace(0, 1, num=1000)
smoothed_positions = np.array([pos(new_t) for pos in position_splines]).T
smoothed_quaternions = np.array([quat(new_t) for quat in quaternion_splines]).T

# 将平滑后的数据写入新的txt文件
with open(output_path, 'w') as f:
    for i in range(len(new_t)):
        f.write(f"{new_t[i]:.6f} {smoothed_positions[i][0]:.6f} {smoothed_positions[i][1]:.6f} {smoothed_positions[i][2]:.6f} {smoothed_quaternions[i][0]:.6f} {smoothed_quaternions[i][1]:.6f} {smoothed_quaternions[i][2]:.6f} {smoothed_quaternions[i][3]:.6f}\n")

print(f"Smoothed data saved to {output_path}")
