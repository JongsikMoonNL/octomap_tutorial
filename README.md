# octomap_tutorial
```bash
sudo apt-get install ros-melodic-octomap
```
이후 
```bash
roscore
rosbag play "filename"
roslaunch fcToTf octomap.launch
rosrun rviz rviz -f world
```
rviz에서 add -> marker array
marker topic은 occupied_cells_vis_array선택
