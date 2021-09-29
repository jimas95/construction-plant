#!/usr/bin/env python

import random
import rospy
from tf.transformations import quaternion_from_euler
from math import pi,cos,sin
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class CANDLE():


    def __init__(self):

        self.candle_size = {"r":0.03,"l":0.01}
        self.arrow_size =  {"w":0.01,"l":0.1}
        self.polar = {"r":1,"th":0}
        self.pose_cartesian = {"x":0,"y":0,"z":0,"roll":0,"pith":0,"yaw":0}
        self.pose = [] 
        self.quad = quaternion_from_euler(1.5707, 0, -1.5707)

        topic = 'candle_marker_array'
        self.publisher = rospy.Publisher(topic, MarkerArray,queue_size=10)

        self.markerArray = MarkerArray()
        self.markerArray.markers.append(Marker())
        self.markerArray.markers.append(Marker())
        self.markerArray.markers.append(Marker())
        self.count = 0
        self.MARKERS_MAX = 10


    def new_candle(self):
        rospy.logdebug("CANDLE--> creating new Candle position")
        self.random_pose()
        self.visualize_candle()
        self.visualize_pregrasp()
        self.visualize_grasp()
        self.publish_visualize()      
        

    def random_pose(self):
        # self.polar['r'] = random.random()/2.0
        self.polar['th'] = 2*pi*random.random()
        self.update_numbers()

    def update_numbers(self):
        self.pose_cartesian['x']    = self.polar['r']*cos(self.polar['th'])
        self.pose_cartesian['y']    = self.polar['r']*sin(self.polar['th'])
        self.pose_cartesian['z']    = self.candle_size['l']/2.0
        self.pose_cartesian['roll'] = 0
        self.pose_cartesian['pitch']= 0 # this should be the grasping direction of the gripper
        self.pose_cartesian['yaw']  = self.polar['th'] #this should be the direction the robot looks at

    def visualize_candle(self):

        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = self.candle_size['r']*2
        marker.scale.y = self.candle_size['r']*2
        marker.scale.z = self.candle_size['l']
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.pose_cartesian['x']
        marker.pose.position.y = self.pose_cartesian['y']
        marker.pose.position.z = self.pose_cartesian['z']
        marker.id = 0

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        # if(self.count > self.MARKERS_MAX):
            # self.markerArray.markers.pop(0)

        # self.markerArray.markers.append(marker)
        self.markerArray.markers[0] = marker
        self.publish_visualize()

    def visualize_pregrasp(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = self.arrow_size['l']
        marker.scale.y = self.arrow_size['w']
        marker.scale.z = self.arrow_size['w']
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.x = quaternion_from_euler(0, 0, self.pose_cartesian['yaw'])[0]
        marker.pose.orientation.y = quaternion_from_euler(0, 0, self.pose_cartesian['yaw'])[1]
        marker.pose.orientation.z = quaternion_from_euler(0, 0, self.pose_cartesian['yaw'])[2]
        marker.pose.orientation.w = quaternion_from_euler(0, 0, self.pose_cartesian['yaw'])[3]
        marker.pose.position.x = self.pose_cartesian['x']
        marker.pose.position.y = self.pose_cartesian['y']
        marker.pose.position.z = self.pose_cartesian['z'] + 0.02
        marker.id = 1

        self.markerArray.markers[1] = marker
        self.count += 1

    def visualize_grasp(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = self.arrow_size['l']
        marker.scale.y = self.arrow_size['w']
        marker.scale.z = self.arrow_size['w']
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.x = quaternion_from_euler(0, pi/4, self.pose_cartesian['yaw'])[0]
        marker.pose.orientation.y = quaternion_from_euler(0, pi/4, self.pose_cartesian['yaw'])[1]
        marker.pose.orientation.z = quaternion_from_euler(0, pi/4, self.pose_cartesian['yaw'])[2]
        marker.pose.orientation.w = quaternion_from_euler(0, pi/4, self.pose_cartesian['yaw'])[3]
        marker.pose.position.x = self.pose_cartesian['x']
        marker.pose.position.y = self.pose_cartesian['y']
        marker.pose.position.z = self.pose_cartesian['z']
        marker.id = 2
        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        # if(self.count > self.MARKERS_MAX):
            # self.markerArray.markers.pop(0)

        # self.markerArray.markers.append(marker)
        self.markerArray.markers[2] = marker

        # Renumber the marker IDs
        # id = 0
        # for m in self.markerArray.markers:
        #     m.id = id
        #     id += 1
        self.count += 1
        self.publish_visualize()

    def publish_visualize(self):
        # Publish the MarkerArray
        self.publisher.publish(self.markerArray)

    def get_pose(self):
        return self.pose

    def print_coord(self):
        pass



