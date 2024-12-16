# RL24_homework3

## First task

### Reguarding the first point of the HM3, it's necessary to check what is the envinroment spawning in the gazebo world (we need to spawn that one with the sphere) (it would also be cool to specify the choosen gazebo world by some parameter passed trough the launch). When the launch file is configured like we want we can start it by the command:
- ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true
### So the simulation starts in gazebo and with the camera being mounted correctly.

### Then to check the correct execution of the first task of the HM3 we need to start the ros2 node that uses openCV to detect che blue sphere and shows the result. This node can be launched by the command:
- ros2 run ros2_opencv ros2_opencv_node
### Once the node starts, a small image will be displayed and if the camera sees the sphere it will be present in the image and surrounded by a red circle (this will be the sign that the object is correcly detected.

## Second task

### I didn't accomplish so much in the second task, but i did the recognition of the arucotag via the camera. So for proving this, firstly the iiwa launch file must be modified so the world in which is spawned the iiwa contains also the arucotag and then the simulation in gazebo can be started (the same command as for the first task). Then we also need to launch the other nodes that are available on our RL24 github page (like the one for the identification of the arucotag, and the ones to use the camera).
