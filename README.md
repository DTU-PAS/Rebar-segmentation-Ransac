Build:
  call "catkin_make" in the repositories root folder

Run:
5 CLI Windows
1. roscore
2. rviz -d default.rviz
3. rosrun rqt_reconfigure rqt_reconfigure
4. roslaunch realsense2_camera rs_camera.launch align_depth:=True filters:=pointcloud
5. roslaunch OnlinePotholeDetection OnlinePotholeDetection.launch
    
