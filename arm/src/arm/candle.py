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

class CANDLE():


    def __init__(self):

        self.candle_size = Vector3(x=0.035,y=0.035,z=0.01)  # x,y diameter z  height
        self.arrow_size  = Vector3(x=0.025,y=0.01,z=0.01)    # x  length   y,z size

        self.colorBlue     = ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0) # Blue 
        self.colorRed      = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0) # Red
        self.colorYellow   = ColorRGBA(r=1.0,g=1.0,b=0.0,a=1.0) # Yellow
        self.colorGreen    = ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0) # Green


        self.polar = {"r":1,"th":0}
        self.offset_grasp = [-0.05,0.05]
        self.offset_pregrasp = [-0.1,0.1]


        topic = 'candle_marker_array'
        self.publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)

        self.markerArray = MarkerArray()
        self.markerArray.markers.append(Marker())
        self.markerArray.markers.append(Marker())
        self.markerArray.markers.append(Marker())
        self.markerType = {"CYLINDER": 3, "ARROW":0}
        

    def new_candle(self):
        rospy.logdebug("CANDLE--> creating new Candle position")
        self.random_pose()
        self.publish_visualize()      
        

    def random_pose(self):
        # self.polar['r'] = random.random()/2.0
        self.polar['theta'] = 2*pi*random.random()
        

    def get_candle_point(self,offset):
        x  = (self.polar['r']+offset[0])*cos(self.polar['theta'])
        y  = (self.polar['r']+offset[0])*sin(self.polar['theta'])
        z  = self.candle_size.z/2.0 + +offset[1]
        point = Point(x=x,y=y,z=z)
        return point

    def add_marker(self,type,size,color,pose,id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = type
        marker.action = marker.ADD
        marker.scale = size
        marker.color = color
        marker.pose = pose 
        marker.id = id
        self.markerArray.markers[id] = marker    

    def publish_visualize(self):
        # add candle marker
        self.add_marker(    type=self.markerType['CYLINDER'],
                            size=self.candle_size,
                            color=self.colorYellow,
                            pose=self.get_candle_pose(),
                            id=0)
        # add grasp pose arrow
        self.add_marker(    type=self.markerType['ARROW'],
                            size=self.arrow_size,
                            color=self.colorBlue,
                            pose=self.get_grasp_pose(),
                            id=1)
        # add pre grasp pose arrow
        self.add_marker(    type=self.markerType['ARROW'],
                            size=self.arrow_size,
                            color=self.colorRed,
                            pose=self.get_pregrasp_pose(),
                            id=2)


        # Publish the MarkerArray
        self.publisher.publish(self.markerArray)


    def get_candle_pose(self):
        return self.get_pose(offset=[0,0],euler=[0,0,0])

    def get_grasp_pose(self):
        direction = [0, pi/4, self.polar['theta']]
        return self.get_pose(offset=self.offset_grasp,euler=direction)

    def get_pregrasp_pose(self):
        direction = [0, pi/4, self.polar['theta']]
        return self.get_pose(offset=self.offset_pregrasp,euler=direction)

    def get_pose(self,offset,euler):
        pos = self.get_candle_point(offset)
        quad = quaternion_from_euler(euler[0],euler[1],euler[2])
        ori = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
        pose = Pose(position = pos, orientation=ori)
        return pose

