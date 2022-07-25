## Easy Camera Lidar Calibration
Repository for rough and easy calibration between camera and lidar through PnP and manual point selection, which can then be used as an initial estimate for other calibration packages.

### Configuration
See the `config/config.yaml` and adjust filenames, ros topics, and camera parameters.

### Usage
First, convert your pointcloud and image messages to a pcd and image files:
```
roslaunch easy_camera_lidar_calibration lidar2pcd.launch
```

In another termanal, play your rosbag:
```
rosbag play your_bag_file.bag --clock
```
Once the bag file finishes playing, press Ctrl+C to save the pcd and image files.

### Calibration
Run the following launch file to start the calibration:
```
roslaunch easy_camera_lidar_calibration calibration.launch
```
When prompted, select the same common points in the 3D pointcloud and the 2D images in the same order. Then, the calibration will be saved.
