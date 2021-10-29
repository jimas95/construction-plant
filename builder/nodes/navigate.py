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
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse


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
        self.odom_frame = 'world'
        self.base_frame = 'base_footprint'

        self.hunt_pt = Vector3()

        rospy.Service('/Navigate/ReverseMode', SetBool, self.setReverseMode)

        self.reverse = False

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)


        # wait for transforms to be published
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame  , rospy.Time(), rospy.Duration(5.0))
            self.tf_listener.waitForTransform(self.base_frame, 'hunt_point'     , rospy.Time(), rospy.Duration(5.0))
            self.tf_listener.waitForTransform('base_footreverse', 'hunt_point'  , rospy.Time(), rospy.Duration(5.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_footprint or for hunt point")
            rospy.signal_shutdown("tf Exception")


    def setReverseMode(self,SetBoolRequest):
        self.reverse = SetBoolRequest.data
        rospy.logdebug("NAVIGATE --> activate reverse mode")
        if(SetBoolRequest.data):
            self.base_frame = 'base_footreverse'
            return SetBoolResponse(success = True,message = "Reverse mode ON")
        self.base_frame = 'base_footprint'
        return SetBoolResponse(success = True,message = "Reverse mode OFF")
        

    def get_tf(self):
        try:
            self.trans = self.tfBuffer.lookup_transform(self.base_frame, 'hunt_point', rospy.Time())
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
        rospy.logdebug(f"NAVIGATION pou prepei na koitaw --> {self.dir*TO_DEGREE:.2f}")
        rospy.logdebug(f"NAVIGATION pou tou lew na koita --> {euler_from_quaternion(poutsa)[2]*TO_DEGREE:.2f}")
        rospy.logdebug(f"NAVIGATION target dist          --> {self.dist:.4f}")
        rospy.logdebug(f"NAVIGATION reverse MODE         --> {self.reverse}")


    def update(self):

        move_cmd = Twist()
        (position, rotation) = self.get_odom()
        self.linear_speed = 1
        self.angular_speed = 1

        while not rospy.is_shutdown():



            self.get_tf()
            self.get_dist()
            self.get_dir()
            self.ti_fasi()

            # poutsa = [self.trans.transform.rotation.x,self.trans.transform.rotation.y,self.trans.transform.rotation.z,self.trans.transform.rotation.w]
            # angle = euler_from_quaternion(poutsa)[2]*TO_DEGREE

            # move_cmd.angular.z = -angle/10
            # move_cmd.linear.x  = self.dist


            # move_cmd.angular.z = 0.5

            move_cmd.angular.z = self.dir
            if(abs(self.dir*TO_DEGREE)<15):
                move_cmd.linear.x = 0.04

            if(self.dist<0.05):
                move_cmd.linear.x = 0.0

            self.threshold_speed(move_cmd)


            # hack for line 
            # hack for line 
            # hack for line 
            # hack for line 
            # (position, rotation) = self.get_odom()
            # if position.x > 0.2 : # go reverse 
            #     self.reverse = True
            # elif(position.x <0):
            #     self.reverse = False

            # move_cmd.linear.x = 0.04
            if(self.reverse):
                move_cmd.linear.x = -move_cmd.linear.x

            


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