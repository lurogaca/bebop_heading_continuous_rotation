# bebop_heading_continuous_rotation
* In this program, the drone will traslate with continuous heading rotation
* In order to run the Autonomous Program, two networks are required in order to connect to the motion capture system and drone, plug in USB WIFI Adapter ( we used LYNEC 150 Mbps)

-> Connect to the Optitrack network  

-> Connect to the drone wifi network

**Open a new terminal : Run ROS**

```
$ roscore
```
**Open a new terminal : Run Mocap** 

```
$ source /opt/ros/noetic/setup.bash
$ roslaunch mocap_optitrack mocap.launch
```
**Open a new terminal : Connect to Drone**

```
$ source /home/labuser/bebop_ws/devel/setup.bash
$ roslaunch bebop_driver bebop_node.launch
```
**Open a new terminal : Launch controller**

```
$ source /home/labuser/bebop_ws/devel/setup.bash
$ roslaunch bebop_tools joy_teleop.launch
```
**Open a new terminal : Run Script**

```
$source /home/labuser/bebop_ws/devel/setup.bash
$rosrun bebop_tools challenge_one.py
```
-> Take off drone by pressing RB + Y

-> Initialize program by pressing 's' + enter 

-> When drone stabalizes, land drone by pressing RB + A

