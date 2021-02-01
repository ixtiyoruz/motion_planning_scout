# motion_planning_scout

<pre>
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_bringup scout_minimal.launch
roslaunch velodyne_pointcloud VLP-32C_points.launch frame_id:=base_link

in order to set the location manually change the fixed frame to a map (global_frame_id). Otherwise it doesnt work.

</pre>

