#!/usr/bin/env python

import random
import re
import rospy
from tf.transformations import quaternion_from_euler
from math import pi,cos,sin
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3,Pose,Point, Quaternion
from std_msgs.msg import ColorRGBA
from arm.srv import get_eef_goal,get_eef_goalResponse,get_eef_goalRequest
from nav_msgs.msg import Path

ARROW_SIZE  = Vector3(x=0.3,y=0.05,z=0.05)    # x  length   y,z size
COLOR_Blue     = ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0) # Blue 
COLOR_Red      = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0) # Red
COLOR_Yellow   = ColorRGBA(r=1.0,g=1.0,b=0.0,a=1.0) # Yellow
COLOR_Green    = ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0) # Green
COLOR_Purple   = ColorRGBA(r=1.0,g=0.0,b=1.0,a=1.0) # Green


class HUNT_POINT():


    def __init__(self):

        self.polar = {"r":1.0,"th":0}
        self.offset_grasp =     [-0.05, 0.028]
        self.offset_pregrasp =  [-0.10, 0.05]


        self.publisherVis = rospy.Publisher("hunt_point_vis" , MarkerArray,queue_size=10)
        self.publisherPose = rospy.Publisher("hunt_point_pose", Pose,queue_size=10)

        self.markerArray = MarkerArray()
        self.markerArray.markers.append(Marker())
        self.markerType = {"CYLINDER": 3, "ARROW":0}
        
        # rospy.Service('/get_eef_goal_'+str(status), get_eef_goal, self.srvf_get_pose)


    def random_pose(self):
        rospy.logdebug("CANDLE--> creating new Candle position")
        # self.polar['r'] = 0.1+random.random()*0.1
        # self.polar['th'] = pi/2*(random.random())
        self.polar['th'] = self.polar['th'] + pi/20
        self.publish_visualize()      
        
    """
    publish the global plan
    """
    def publish_plan(self):
        msg = Path()
        msg.header.frame_id = "/odom"
        msg.header.stamp = rospy.Time.now()
        pose = self.get_pose()
        msg.poses.append(pose)
        rospy.loginfo("Publishing Plan...")
        self.waypoint_publisher.publish(msg) 

    def get_candle_point(self,offset):
        x  = (self.polar['r']+offset[0])*cos(self.polar['th'])
        y  = (self.polar['r']+offset[0])*sin(self.polar['th'])
        z  = 0 + offset[1]
        point = Point(x=x,y=y,z=z)
        return point

    def add_marker(self,type,size,color,pose,id):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = type
        marker.action = marker.ADD
        marker.scale = size
        marker.color = color
        marker.pose = pose 
        marker.id = id
        self.markerArray.markers[id] = marker    

    def publish_visualize(self):
        # add arrown at hunt point
        self.add_marker(    type=self.markerType['ARROW'],
                            size=ARROW_SIZE,
                            color=COLOR_Blue,
                            pose=self.get_pose(),
                            id=0)

        # Publish the MarkerArray
        self.publisherVis.publish(self.markerArray)
        self.publisherPose.publish(self.get_pose())


    def get_pose(self):
        offset=[0,0]
        euler=[0,0,self.polar['th']+pi/2]
        pos = self.get_candle_point(offset)
        quad = quaternion_from_euler(euler[0],euler[1],euler[2])
        ori = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
        pose = Pose(position = pos, orientation=ori)
        return pose


    """Service get eef goal
    Empty request
    returns the geometry pose of grasp and pregrasp pos
    """
    def srvf_get_pose(self,get_eef_goalRequest):
        msg = get_eef_goalResponse()
        msg.grasp = self.get_grasp_pose()
        msg.pregrasp = self.get_pregrasp_pose()
        return msg

""" INIT    """
def start():

    rospy.init_node('Path_Planning', log_level=rospy.DEBUG)
    rate = rospy.Rate(1.0) # publish freacuancy 
    hunt_point = HUNT_POINT()

    while not rospy.is_shutdown():
        rospy.logdebug("Path_Planning --> loopa")
        hunt_point.random_pose()
        hunt_point.publish_visualize()
        rate.sleep()

if __name__ == '__main__':
    try:
        start() 
    except rospy.ROSInterruptException:
        pass
