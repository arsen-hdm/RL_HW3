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

### Sending position commands
For last, if you want also to send some position commands to the robot so that it moves as desired, you must run the previous command but also:
```bash
ros2 run arm_control control_node
```
Remember that if you want to change the desired positions of the joints you need to do simple modifications in the node src code of the arm_control package. The part of code is specified in the presentation with also the path to it.
Thanks for the attention, you can find also other homeworks in my personal repositories!
