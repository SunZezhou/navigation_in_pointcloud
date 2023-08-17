# navigation_in_pointcloud
First-person view navigation in point clouds using Open3d

![demo](https://github.com/SunZezhou/navigation_in_pointcloud/assets/49157013/c396f577-61dc-4d50-bbc4-0297f9696287)

## Prerequisite

- `pip install open3d`

## Usage

1. `bessel_curve_interpolation.py`: Interpolating the camera pose in /data/viewpoints.txt using Bézier curves.

1. `first-view-navigation.py`: Displays the point cloud in first-person view according to the camera position.

1. (option) `save_video.py`: Generate full navigation videos.

1. (option) `line_interpolation.py`: Linear interpolation instead of Bezier curve interpolation
