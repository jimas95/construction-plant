#!/usr/bin/env python
#################################################################################
#################################################################################

# Authors: D.Chamzas #

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose, Vector3
import tf,tf2_ros
from math import radians, copysign, sqrt, pow, pi, atan2,cos,sin
from tf.transformations import euler_from_quaternion
import numpy as np
import random

"""
NAVIGATE your Turtlebot3!

"""

MAX_ROTATION_SPEED = 1.5
MAX_LINEAR_SPEED = 0.5
TO_DEGREE = 57.2958

class GotoPoint():
    def __init__(self):
        rospy.init_node('navigate',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.hunt_pt = Vector3()

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # wait for transforms to be published
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint'  , rospy.Time(), rospy.Duration(2.0))
            self.tf_listener.waitForTransform("base_footprint", 'hunt_point'     , rospy.Time(), rospy.Duration(2.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_footprint or for hunt point")
            rospy.signal_shutdown("tf Exception")


    def get_tf(self):
        try:
            self.trans = self.tfBuffer.lookup_transform("base_footprint", 'hunt_point', rospy.Time())
            # self.trans = self.tfBuffer.lookup_transform("hunt_point","base_footprint", rospy.Time())
            # rospy.logdebug(mk)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS ")
            self.r.sleep()

    def get_dist(self):
        self.dist = sqrt(self.trans.transform.translation.x ** 2 + self.trans.transform.translation.y ** 2)

    def get_dir(self):
        self.dir = atan2(self.trans.transform.translation.y, self.trans.transform.translation.x)

    def reach_pos(self):
        if self.dist > 0.05:
            return False
        return True

    def reach_dir(self):
        pass

    def set_cmd(self):
        pass

    def ti_fasi(self):
        # rospy.logdebug(f"NAVIGATION DIST--> {self.dist:.2f}")
        # rospy.logdebug(f"NAVIGATION DIR --> {self.dir*TO_DEGREE:.2f}")
        (position, rotation) = self.get_odom()
        # rospy.logdebug(f"NAVIGATION ROT --> {rotation*TO_DEGREE}")
        # rospy.logdebug(f"NAVIGATION TRA --> {self.trans.transform}")

        poutsa = [self.trans.transform.rotation.x,self.trans.transform.rotation.y,self.trans.transform.rotation.z,self.trans.transform.rotation.w]

        rospy.logdebug(f"------ NAVIGATION ------")
        rospy.logdebug(f"NAVIGATION pou koitaw           --> {rotation*TO_DEGREE:.2f}")
        # rospy.logdebug(f"NAVIGATION pou error error      --> {self.dir*TO_DEGREE:.2f}")
        rospy.logdebug(f"NAVIGATION pou prepei na koitaw --> {self.dir*TO_DEGREE:.2f}")
        rospy.logdebug(f"NAVIGATION pou tou lew na koita --> {euler_from_quaternion(poutsa)[2]*TO_DEGREE:.2f}")
        rospy.logdebug(f"NAVIGATION plaka me k           --> {self.trans.transform.translation.x:.2f}")
        rospy.logdebug(f"NAVIGATION plaka me k           --> {self.trans.transform.translation.y:.2f}")


    def update(self):

        position = Point()
        move_cmd = Twist()
        (position, rotation) = self.get_odom()
        self.last_rotation = 0
        self.linear_speed = 1
        self.angular_speed = 1

        while not rospy.is_shutdown():



            self.get_tf()
            self.get_dist()
            self.get_dir()
            self.ti_fasi()



            # move_cmd.angular.z = 4 * atan2(self.trans.transform.translation.y, self.trans.transform.translation.x)
            # move_cmd.linear.x = 0.5 *sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            # move_cmd.linear.x = 0.0
            # move_cmd.angular.z = 0.5
            move_cmd.angular.z = self.dir
            

            self.threshold_speed(move_cmd)


            # rospy.logdebug(f"NAVIGATION --> {move_cmd}")
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
            rospy.loginfo("NAVIGATION --> TF Exception")
            return
        return (Point(*trans), rotation[2])

    """
    callback function for shutdown of node
    """
    def shutdown(self):
        rospy.logerr("NAVIGATION --> CLOSING")
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
        rospy.logerr("NAVIGATION --> SHUT DOWN")
        rospy.logerr("NAVIGATION --> SHUT DOWN")