#!/usr/bin/env python
#################################################################################
#################################################################################

# Authors: D.Chamzas #

import rospy
from geometry_msgs.msg import Twist, Vector3
import tf2_ros
from math import sqrt, pi, atan2
from std_srvs.srv import SetBool,SetBoolResponse


"""
NAVIGATE your Turtlebot3!

"""

MAX_ROTATION_SPEED = 1.5
MAX_LINEAR_SPEED = 0.5
MIN_DIST_THRESHOLD = 0.05
MIN_DIR_THRESHOLD = 10
TO_DEGREE = 180.0/pi

class GotoPoint():
    def __init__(self):
        rospy.init_node('navigate',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.r = rospy.Rate(10) # frequency
        self.odom_frame = 'world'
        self.base_frame = 'base_footprint'
        self.hunt_frame = 'hunt_point'

        self.hunt_pt = Vector3()

        rospy.Service('/navigate/reverse_mode', SetBool, self.setReverseMode)

        self.reverse = True

        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
    

        # wait for transforms to be published
        try:
            self.trans = self.tfBuffer.lookup_transform(self.odom_frame   , self.base_frame, rospy.Time(),rospy.Duration(5.0))
            self.trans = self.tfBuffer.lookup_transform(self.base_frame   , self.hunt_frame, rospy.Time(),rospy.Duration(5.0))
            self.trans = self.tfBuffer.lookup_transform("base_footreverse", self.hunt_frame, rospy.Time(),rospy.Duration(5.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS")

        

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
            self.trans = self.tfBuffer.lookup_transform(self.base_frame, self.hunt_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS")
            self.r.sleep()

    """
    return the distance from turtle pose relative to hunting point
    """
    def get_dist(self):
        self.dist = sqrt(self.trans.transform.translation.x ** 2 + self.trans.transform.translation.y ** 2)

    """
    return the direction error from turtle pose relative to hunting point
    """
    def get_dir(self):
        self.dir = atan2(self.trans.transform.translation.y, self.trans.transform.translation.x)


    """
    return True if reached destination goal
    """
    def reach_pos(self):
        if self.dist > MIN_DIST_THRESHOLD:
            return False
        return True
    """
    if within direction margin return True
    if error angle < threshhold return true so we also start moving forward 
    """
    def reach_dir(self):
        return (abs(self.dir*TO_DEGREE)<MIN_DIR_THRESHOLD)

    """
    calculate/return the correct cmd vel
    """
    def get_cmd(self):
        move_cmd = Twist()
        move_cmd.angular.z = self.dir

        # if within direction margin starting going forward
        if(self.reach_dir()):
            move_cmd.linear.x = 0.04

        if(self.reach_pos()):
            move_cmd.linear.x  = 0.0
            move_cmd.angular.z = 0.0

        if(self.reverse):
            move_cmd.linear.x = - move_cmd.linear.x
        
        self.threshold_speed(move_cmd)
        return move_cmd

    def debug_msg(self):
        rospy.logdebug(f"------ NAVIGATION ------")
        rospy.logdebug(f"NAVIGATION direction error      --> {self.dir*TO_DEGREE:.2f}")
        rospy.logdebug(f"NAVIGATION target dist          --> {self.dist:.4f}")
        rospy.logdebug(f"NAVIGATION reverse MODE         --> {self.reverse}")
        rospy.logdebug(f"------------------------")


    def update(self):


        while not rospy.is_shutdown():

            self.get_tf()
            self.get_dist()
            self.get_dir()
            self.debug_msg()

            self.cmd_vel.publish(self.get_cmd())
            self.r.sleep()


        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())


    def threshold_speed(self, move_cmd):
        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z,  MAX_ROTATION_SPEED)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -MAX_ROTATION_SPEED)

        if move_cmd.linear.x > 0:
            move_cmd.linear.x = min(move_cmd.linear.x,  MAX_LINEAR_SPEED)
        else:
            move_cmd.linear.x = max(move_cmd.linear.x, -MAX_LINEAR_SPEED)
        

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