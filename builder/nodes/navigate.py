#!/usr/bin/env python
#################################################################################
#################################################################################

# Authors: D.Chamzas #

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose, Vector3
import tf
from math import radians, copysign, sqrt, pow, pi, atan2,cos,sin
from tf.transformations import euler_from_quaternion
import numpy as np
import random

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

MAX_ROTATION_SPEED = 1.5
MAX_LINEAR_SPEED = 0.5


class GotoPoint():
    def __init__(self):
        rospy.init_node('navigate',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.hunt_pt = Vector3()

        rospy.Subscriber("hunt_point_pose", Pose, self.callback_hunt_point)

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")


    def callback_hunt_point(self,msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
        self.hunt_pt.x = msg.position.x
        self.hunt_pt.y = msg.position.y
        self.hunt_pt.z = msg.position.z


    def update(self):

        position = Point()
        move_cmd = Twist()
        (position, rotation) = self.get_odom()
        self.last_rotation = 0
        self.linear_speed = 1
        self.angular_speed = 1

        while not rospy.is_shutdown():

            (goal_x, goal_y, goal_z) = (self.hunt_pt.x,self.hunt_pt.y,self.hunt_pt.z)
            rospy.logdebug(f"x,y = ({goal_x},{goal_y}) , angle = {goal_z}")


            # goal_z = np.deg2rad(goal_z)
            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            distance = goal_distance

            rospy.logdebug("NAVIGATION --> Looping")
            rospy.logdebug(f"x,y = ({goal_x},{goal_y}) , angle = {goal_z}")

            (position, rotation) = self.get_odom()
            rospy.logdebug(f"pos,rot= ({position},{rotation})")

            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle

            if self.last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif self.last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
                
            move_cmd.angular.z = self.angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(self.linear_speed * distance, 0.5)

            self.threshold_speed(move_cmd)

            if distance < 0.05:
                move_cmd.linear.x = 0

            # move_cmd.linear.x = 0.01
            # move_cmd.angular.z = 0.5
            self.last_rotation = rotation
            rospy.logdebug(f"NAVIGATION --> {move_cmd}")
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()


        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def threshold_speed(self, move_cmd):
        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z,  MAX_ROTATION_SPEED)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -MAX_ROTATION_SPEED)

        move_cmd.linear.x = min(move_cmd.linear.x, MAX_ROTATION_SPEED)
        

    def get_cmd(self):
        move_cmd = Twist()
        goal = {"x":1,"y":0, "th":0}
        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y
        path_angle = atan2(goal['y'] - y_start, goal['x']- x_start)

        if path_angle < -pi/4 or path_angle > pi/4:
            if goal['y'] < 0 and y_start < goal['y']:
                path_angle = -2*pi + path_angle
            elif goal['y'] >= 0 and y_start > goal['y']:
                path_angle = 2*pi + path_angle

        if self.last_rotation > pi-0.1 and rotation <= 0:
            rotation = 2*pi + rotation
        elif self.last_rotation < -pi+0.1 and rotation > 0:
            rotation = -2*pi + rotation
            
        move_cmd.angular.z = self.angular_speed * path_angle-rotation

        distance = sqrt(pow((goal['x'] - x_start), 2) + pow((goal['x'] - y_start), 2))
        move_cmd.linear.x = min(self.linear_speed * distance, 0.5)

        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

        self.last_rotation = rotation 
        return move_cmd



    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    """
    callback function fur shutdown of node
    """
    def shutdown(self):
        rospy.logerr("NAVIGATION CLOSING")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)



"""
creates object of class and calls main loop 
"""
if __name__ == '__main__':
    try:
        robot = GotoPoint()
        robot.update()
    except rospy.ROSInterruptException:
        rospy.loginfo("shutdown navigation node.")
