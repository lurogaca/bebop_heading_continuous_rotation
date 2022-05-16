#!/usr/bin/env python

import rospy
from simple_pid import PID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

import time
import math
import tf

pid_x = PID(0.15, 0, 0.15, setpoint = 0)
pid_y = PID(0.15, 0, 0.15, setpoint = 0)
#pid_yaw = PID(0.1, 0, 0, setpoint = -1.578)

opti_x = 0
opti_y = 0
opti_z = 0
point = 0

x = 0
y = 0
z = 0

yaw_degrees = 0

def heading_comp(actual, waypoint):
    difference = waypoint - actual
    while difference < -180:
        difference += 360
    while difference > 180:
        difference -= 360
    return difference

def constrain_angle(x):
    x = math.fmod(x, 360)
    if(x < 0):
        x += 360
    return x


def odom_callback(msg):
    global x, y, z, constant_rotation, point, opti_x, opti_y, opti_z, yaw_degrees

    pub_bebop_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=2)
    twist_bebop = Twist()

        
    opti_x = msg.pose.pose.position.y
    opti_y = msg.pose.pose.position.x
    opti_z = msg.pose.pose.position.z

    opti_ang_x = msg.pose.pose.orientation.x
    opti_ang_y = msg.pose.pose.orientation.y
    opti_ang_z = msg.pose.pose.orientation.z
    opti_ang_w = msg.pose.pose.orientation.w
    
    quaternion = [opti_ang_x, opti_ang_y, opti_ang_z, opti_ang_w]

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    
    yaw_degrees = math.degrees(yaw)
    #fixed_heading = heading_comp(yaw_degrees, pid_yaw.setpoint)
    yaw = math.radians(yaw_degrees)
    rad_heading = math.radians(yaw_degrees)

    x1 = pid_x(opti_x)
    y1 = pid_y(opti_y)
    z = 0.15
	# z changes the rotation velocity

    error_x = pid_x.setpoint - opti_x
    error_y = pid_y.setpoint - opti_y
    #error_z = pid_yaw.setpoint - yaw
    constrain_heading = constrain_angle((yaw_degrees))
    rad_constrain = math.radians(constrain_heading)
    
    x = x1 * math.cos(rad_constrain) + -1 * y1 * math.sin(rad_constrain)
    y = x1 * math.sin(rad_constrain) + y1 * math.cos(rad_constrain)


    twist_bebop.linear.x = x
    twist_bebop.linear.y = -1 * y
    twist_bebop.angular.z = z
    print("opti x ==> " + str(opti_x))
    print("opti y ==> " + str(opti_y))
    print("control x ==> " + str(x))
    print("control y ==> " + str(y))
    print("control z ==> " + str(z))
    print("=======================================")
    pub_bebop_vel.publish(twist_bebop)

def main():
    # Initialize node
    rospy.init_node('challengeOne', anonymous=True)

    # Subscribing to the mocap_node and the Odometry topic
    rospy.Subscriber('/mocap_node/bebop_x/Odom', Odometry, odom_callback)
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 10)
    	# change name in line 101 to receive data from respective drone 

    rospy.spin()

if __name__ == '__main__':
    starting = input("Press s to start: ")
    if starting == "s":
        print("Starting main()")
        main()
        print("Done!!")
