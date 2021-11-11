#!/usr/bin/env python
#################################################################################
#################################################################################

# Authors: D.Chamzas #

import re
import rospy
from geometry_msgs.msg import Twist, Vector3
import tf2_ros
from math import inf, sqrt, pi, atan2
import actionlib
import builder.msg
import time
from tf.transformations import euler_from_quaternion

"""
NAVIGATE DIFFERENTIAL DRIVE ROBOT

"""

MAX_ROTATION_SPEED = 1.5
MAX_LINEAR_SPEED = 0.5
MIN_DIST_THRESHOLD = 0.05
MIN_DIR_THRESHOLD = 5
TO_DEGREE = 180.0/pi
FREQUENCY = 20

class GotoPoint():
    def __init__(self):
        # rospy.init_node('navigate',log_level=rospy.DEBUG)
        rospy.init_node('navigate',log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)


        self.hunt_pt = Vector3()
        self.reverse = True
        self.aling_htp = False
        self.setReverseMode(self.reverse)

        use_real = rospy.get_param("/use_real")
        self.linearSpeed = 0.04
        if(not use_real):
            self.linearSpeed = 0.1
    
        # create hunting action 
        self._feedback = builder.msg.huntFeedback()
        self._result   = builder.msg.huntResult()
        self._action = actionlib.SimpleActionServer("action_hunt", builder.msg.huntAction, execute_cb=self.execute_action, auto_start = False)
        self._action.start()

        # init & wait for transforms to be published
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.odom_frame = 'world'
        self.base_frame = 'base_footprint'
        self.hunt_frame = 'hunt_point'
        try:
            self.trans = self.tfBuffer.lookup_transform(self.odom_frame   , self.base_frame, rospy.Time(),rospy.Duration(1.0))
            self.trans = self.tfBuffer.lookup_transform(self.base_frame   , self.hunt_frame, rospy.Time(),rospy.Duration(1.0))
            self.trans = self.tfBuffer.lookup_transform("base_footreverse", self.hunt_frame, rospy.Time(),rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS")



    """ 
    EXECUTE ACTION
    callback function of hunting (GOTO) action 
    """
    def execute_action(self, goal):
        self.setReverseMode(goal.reverseMD)
        r = rospy.Rate(FREQUENCY)
        success = False
        self.aling_htp = False
        
        # publish info to the console for the user
        rospy.loginfo("NAVIGATION --> HUNTER ACTION ACTIVATE")

        # self.debug_msg()

        # start executing the action
        while(not success):
            
            # check that preempt has not been requested by the client
            if self._action.is_preempt_requested():
                rospy.logdebug("NAVIGATION --> HUNTER ACTION CANCEL" )
                rospy.loginfo("NAVIGATION --> Stopping the robot...")
                self._action.set_preempted()
                success = True
                self.cmd_vel.publish(Twist())
                break
            
            success = self.update()

            # update & publish the feedback
            self._feedback.error_dist = self.dist
            self._feedback.error_dir  = self.dir*TO_DEGREE
            self._action.publish_feedback(self._feedback)

            if(goal.debugMD):
                time.sleep(0.3)
                success = True

            r.sleep()
          
            if success:
                self._result.success = success
                self._action.set_succeeded(result = self._result)
                rospy.loginfo(f"NAVIGATION --> HUNTER ACTION SUCCESS {success}")


    """
    SERVICE
    change robot base frame, depending on if we navigate on reverse
    base_footreverse frame is the same frame(base_footprint) rotated by 180 degrees
    """
    def setReverseMode(self,reverseMD):
        self.reverse = reverseMD
        if(reverseMD):
            self.base_frame = 'base_footreverse'
            rospy.logdebug("NAVIGATE --> Reverse mode ON")
            return
        self.base_frame = 'base_footprint'
        rospy.logdebug("NAVIGATE --> Reverse mode OFF")

    """
    UPDATE transforamation using tf2
    lookUP from robot base --> hunting point
    """
    def get_tf(self):
        try:
            self.trans = self.tfBuffer.lookup_transform(self.base_frame, self.hunt_frame, rospy.Time(),rospy.Duration(0.1))
        except (tf2_ros.LookupException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS!!")
            rospy.logerr(f"NAVIGATE --> PROBLEMO WITH {self.base_frame}  {self.hunt_frame}")
            rospy.logerr("MAKIS")
            rospy.logerr("-----------------")
        except (tf2_ros.ConnectivityException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS!!")
            rospy.logerr(f"NAVIGATE --> PROBLEMO WITH {self.base_frame}  {self.hunt_frame}")
            rospy.logerr("SAKIS") 
            rospy.logerr("-----------------")
        except (tf2_ros.ExtrapolationException):
            rospy.logerr("NAVIGATE --> PROBLEMO WITH TFS!!")
            rospy.logerr(f"NAVIGATE --> PROBLEMO WITH {self.base_frame}  {self.hunt_frame}")
            rospy.logerr("TAKIS")
            rospy.logerr("-----------------")

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
        quad = (self.trans.transform.rotation.x,self.trans.transform.rotation.y,self.trans.transform.rotation.z,self.trans.transform.rotation.w)
        yaw = euler_from_quaternion(quad)[2]
        if(self.aling_htp):
            self.dir = yaw

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
            move_cmd.linear.x = self.linearSpeed

        if(self.reach_pos()):
            move_cmd.linear.x  = 0.0
            # aling with hunting point
            self.aling_htp = True
            self.get_dir()



        if(self.reverse):
            move_cmd.linear.x = - move_cmd.linear.x
        
        self.threshold_speed(move_cmd)
        return move_cmd

    def debug_msg(self):
        self.get_tf()
        self.get_dist()
        self.get_dir()
        rospy.logerr(f"------ NAVIGATION ------")
        rospy.logerr(f"NAVIGATION direction error      --> {self.dir*TO_DEGREE:.2f}")
        rospy.logerr(f"NAVIGATION target dist          --> {self.dist:.4f}")
        rospy.logerr(f"NAVIGATION target goal          --> {self.trans.transform.translation.x:.4f} {self.trans.transform.translation.y:.4f}")
        rospy.logerr(f"NAVIGATION reverse MODE         --> {self.reverse}")
        rospy.logerr(f"------------------------")

    """
    main navigation functions, make all calculation and publish cmd
    return False only if target goal (hunting point) is reached
    """
    def update(self):
        self.get_tf()
        self.get_dist()
        self.get_dir()
        self.cmd_vel.publish(self.get_cmd())
        # self.debug_msg()

        if(self.reach_dir() and self.reach_pos()):
            return True

        return False
        


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
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("NAVIGATION --> SHUT DOWN")
        rospy.logerr("NAVIGATION --> SHUT DOWN")