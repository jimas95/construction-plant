#! /usr/bin/env python
import rospy
from math import *
import numpy as np
from nav_msgs.msg import Odometry
import tf
import tf2_ros
from geometry_msgs.msg import Vector3,Pose,Point, Quaternion,Transform,TransformStamped,PoseStamped
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

last_odom = None
pose = [0.0,0.0,0.0]
a1 = 0.0
a2 = 0.0
a3 = 0.0
a4 = 0.0
new_odom_frame = ""
odom_frame = ""
 
def callback(data):
    global last_odom
    global new_odom_frame
    global odom_frame
    global pose
    global a1
    global a2
    global a3
    global a4
 
    q = [ data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w ]
    (r, p, theta2) = tf.transformations.euler_from_quaternion(q)
 
    if(last_odom == None):
        last_odom = data
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y
        pose[2] = theta2
    else:
        dx = data.pose.pose.position.x - last_odom.pose.pose.position.x
        dy = data.pose.pose.position.y - last_odom.pose.pose.position.y
        # rospy.logerr(dx)
        # rospy.logerr(dy)
        # rospy.logerr("")

        trans = sqrt(dx*dx + dy*dy)
        q = [ last_odom.pose.pose.orientation.x,
                last_odom.pose.pose.orientation.y,
                last_odom.pose.pose.orientation.z,
                last_odom.pose.pose.orientation.w ]
        (r,p, theta1) = tf.transformations.euler_from_quaternion(q)
        rot1 = atan2(dy, dx) - theta1
        rot2 = theta2-theta1-rot1
        # rospy.logerr(trans)
        # rospy.logerr(rot1)
        # rospy.logerr(rot2)
 
        sd_rot1 = a1*abs(rot1) + a2*trans
        sd_rot2 = a1*abs(rot2) + a2*trans
        sd_trans = a3*trans + a4*(abs(rot1) + abs(rot2))
 
        trans +=  np.random.normal(0,sd_trans*sd_trans)
        rot1 += np.random.normal(0, sd_rot1*sd_rot1)
        rot2 += np.random.normal(0, sd_rot2*sd_rot2)
 
        pose[0] += trans*cos(theta1+rot1)
        pose[1] += trans*sin(theta1+rot1)
        pose[2] +=  rot1 + rot2
        last_odom = data
    
    rospy.logerr(pose[0])
    rospy.logerr(pose[1])
    rospy.logerr(pose[2])
 
    pos = Vector3(x = pose[0] - data.pose.pose.position.x, y =pose[1] - data.pose.pose.position.y, z = 0)
    quad = quaternion_from_euler(0, 0, pose[2] - theta2)
    ori = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
    hp_tf = Transform(translation = pos, rotation=ori)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_footprint_real"
    t.child_frame_id = "base_footprint"
    t.transform = hp_tf
    br.sendTransform(t)  
    rospy.logerr("ti fasi ?")

def callback_cmd(data):
    global pose 

    linear = data.linear.x
    angular = data.angular.z
    
    sd_rot1 = 0 #angular*1 + linear*0.5
    sd_trans = 0 #linear*1.2

    trans = np.random.normal(0,sd_trans*sd_trans)
    rot1  = np.random.normal(0, sd_rot1*sd_rot1)

    pose[0] += trans*cos(rot1) #+ 0.01/200
    pose[1] += trans*sin(rot1)
    pose[2] +=  rot1 
    
    rospy.logerr(pose[0])
    rospy.logerr(pose[1])
    rospy.logerr(pose[2])
 
    pos = Vector3(x = pose[0] , y =pose[1] , z = 0)
    quad = quaternion_from_euler(0, 0, pose[2])
    ori = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
    hp_tf = Transform(translation = pos, rotation=ori)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_footprint_real"
    t.child_frame_id = "base_footprint"
    t.transform = hp_tf
    br.sendTransform(t)  


if __name__ == '__main__':
    rospy.init_node('noisy_odometry', anonymous=True)

    a1 = 0.05
    a2 = 10.0*pi/180.0
    a3 = 1.1
    a4 = 0.01
    odom_topic = "/odom"
    new_odom_frame = "odom"
    odom_frame = "odom"
 
    # rospy.Subscriber(odom_topic, Odometry, callback)
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd)
    rospy.spin()