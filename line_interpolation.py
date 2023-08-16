import numpy as np
from scipy.spatial.transform import Rotation as R

def interpolate_pose(pose1, pose2, num_points):
    timestamp1, translation1, rotation_quat1 = pose1
    timestamp2, translation2, rotation_quat2 = pose2
    
    interpolated_poses = []
    translation_diff = np.array(translation2) - np.array(translation1)
    
    for i in range(num_points + 1):
        t = i / num_points
        interpolated_translation = np.array(translation1) + t * translation_diff
        
        # Keep rotation as is
        interpolated_rotation_quat = rotation_quat1
        
        interpolated_poses.append((timestamp1 + t * (timestamp2 - timestamp1), interpolated_translation, interpolated_rotation_quat))
    
    return interpolated_poses

def read_tum_file(file_path):
    poses = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for i in range(0, len(lines) - 1, 2):
            parts1 = lines[i].split()
            timestamp1 = float(parts1[0])
            translation1 = [float(parts1[j]) for j in range(1, 4)]
            rotation_quat1 = np.array([float(parts1[j]) for j in range(4, 8)])
            
            parts2 = lines[i + 1].split()
            timestamp2 = float(parts2[0])
            translation2 = [float(parts2[j]) for j in range(1, 4)]
            rotation_quat2 = np.array([float(parts2[j]) for j in range(4, 8)])
            
            poses.append(((timestamp1, translation1, rotation_quat1), (timestamp2, translation2, rotation_quat2)))
            
    return poses

def main():
    input_file_path = "./data/viewpoints.txt"
    output_file_path = "./data/viewpoints_line_interpolation.txt"

    pose_pairs = read_tum_file(input_file_path)
    modified_poses = []

    num_interpolated_points = 100

    for pose1, pose2 in pose_pairs:
        interpolated_poses = interpolate_pose(pose1, pose2, num_interpolated_points)
        
        for interpolated_pose in interpolated_poses:
            modified_pose = np.concatenate(([interpolated_pose[0]], interpolated_pose[1], interpolated_pose[2]))
            modified_poses.append(modified_pose)

    with open(output_file_path, 'w') as f:
        for modified_pose in modified_poses:
            formatted_pose = " ".join([f"{value:.6f}" for value in modified_pose])
            f.write(f"{formatted_pose}\n")

if __name__ == "__main__":
    main()
