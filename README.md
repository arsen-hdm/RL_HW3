# HM3 RL 24/25 Arsen Hudyma

Hi, this is the repository for the HM3 of the course 'Robotics Lab'. In this homework we're be using as before ros2 and gazebo to spawn the already presented iiwa manipulator, but also we'll use some world models for gazebo to visualize a blue spherical element and visualize it thought the camera sensor mounted on the EE of the robot. And that's not all, because we'll be also using openCV functionalities to implement a node that practically will do a task of blob (or object) detection thought the data acquired from the camera sensor.

To start, you'll need this repository on your computer, so get it by:
```bash
git clone https://github.com/arsen-hdm/RL_HM3.git
```
Apart of the repository, give a look at my presentation of this HW so that it can be more clear and for better understanding of the things I've done.
Consider that it's a powerpoint and in some parts it may contain videos, so remember to look at it ;)

Then, once you're in the dockek container, firstly:
```bash
colcon build
. install/setup.bash
```

### Simulation in gazebo, with the camera being mounted
In this homework we're supposed to mount a camera on the robot, but in function of the boolean parameter *use_vision*, since it's a vision oriented task we'll need always the camera being active.
The command to start the simulation is:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true
```
We can be sure that the camera is working simply adding the image display functionality in gazebo (remember to start effectively the simulation by the play button in the lower left of the gazebo window).

'An important thing to remember' is to know which is the world loaded by gazebo, we can see it in the launch file of the iiwa (the iiwa.launch.py), and notice also in the presentation of this homework.
The world with the arucotag is the *empty.world* and the one with the blue sphere is *empty_and_a_sphere.world*. In this case we need the second one (for now).

### Running the blob detector
After starting the iiwa we need to start the blob detector node, once started a small window will pop-up, if the desired object is recognized we'll see a red circle around it.
Try to impose the circularity of the detected object to zero, if everything will work fine the sphere will not me detected, becayse of its form ;)
However, the command to start the node is this one:
```bash
ros2 run ros2_opencv ros2_opencv_node
```

For now it'all but soon I'll add the other part of this HW and the last one, the HW4. See you soon" ;)

### Using the aruco tag
For the second part of the HW you have to load a gazebo world containg an aruco tag, for changing the spawning world you must go in the iiwa.launch.py file, give a look at the ppt of this homework to see how to do it simply ;)
Once you modified the launch file, firstly you must:
```bash
colcon build
. install/setup.bash
```
Then the command to load the iiwa is basically the same, so:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true
```
Then you have to launch the usb cam executable, in my case I launch it with the default parameter that you should have, otherwise use your way if it works :) Mine command is:
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file src/ros2_vision/config/camera_params.yaml -r /image_raw:=/stereo/left/image_rect_color -r /camera_info:=/stereo/left/camera_info
```
Then start the simple aruco node with the launch file:
```bash
ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201
```
Mine doesn't work properly sometimes so if you want to see it working check my presentation, hope it helps.

If the aruco detector works properly then you can try to start the kdl vision node, by default its set to velocity commands and positioning task (also the only one implemented.
So give it a try and if it works let me know!
```bash
ros2 run ros2_kdl_package ros2_kdl_vision
```

Thanks for the attention, you can find also other homeworks in my personal repositories! Goodbye and to the next one!
