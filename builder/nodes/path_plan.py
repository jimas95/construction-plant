#!/usr/bin/env python

import random
import rospy
from tf.transformations import quaternion_from_euler
from math import pi,cos,sin,sqrt
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3,Pose,Point, Quaternion,Transform,TransformStamped,PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf_conversions
import tf2_ros


ARROW_SIZE  = Vector3(x=0.3,y=0.05,z=0.05)    # x  length   y,z size
COLOR_Blue     = ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0) # Blue 
COLOR_Red      = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0) # Red
COLOR_Yellow   = ColorRGBA(r=1.0,g=1.0,b=0.0,a=1.0) # Yellow
COLOR_Green    = ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0) # Green
COLOR_Purple   = ColorRGBA(r=1.0,g=0.0,b=1.0,a=1.0) # Green

TO_DEGREE = 57.2958

MODE = "CIRCLE"
MODE = "LINE"

class HUNT_POINT():


    def __init__(self):

        self.polar  = {"r":0.2,"th":0}
        self.center = {"x":0.5,"y":0} # offset of circle center
        self.rangeUPDOWN = 0.25
        self.dir = 0 
        self.steps = 1
        self.step = pi/self.steps
        self.offsetDIR = pi/2
        self.reverse = False
        self.time = 0 

        self.huntPT = Point(x=0,y=0,z=0)


        self.publisherVis  = rospy.Publisher("hunt_point_vis" , MarkerArray,queue_size=10)
        self.publishPath   = rospy.Publisher("hunt_point_path", Path,queue_size=10)
        self.msg_path = Path()
        

        self.markerArray = MarkerArray()
        self.markerArray.markers.append(Marker())
        self.markerType = {"ARROW":0}


        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)


        # create path plan 
        for timeStep in range(2*self.steps+1):
            self.time += self.step
            self.next_hp()
            pose = self.get_pose()
            pose_stamped = PoseStamped(pose = pose)
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "world"
            self.msg_path.poses.append(pose_stamped)
            self.publish_plan()
        
        # reset time
        self.time = 0 
        # self.time = random.random()*2*pi

        # send info tf, plan, arrow got hunting point
        self.publish_visualize()    
        self.publish_huntTF()

    """     main loop    """
    def update(self):
        rospy.logdebug("PATH PLAN--> update")

        if(self.goal_reached()):
            self.time += self.step
            self.reverse = not self.reverse
            self.setReverseMode(self.reverse)
            # every pi switch reverse mode ( for LINE PATH)
            # rospy.logerr(f" condition --> {int(self.time/(pi/2))} {int((self.time-self.step)/(pi/2))}")
            # rospy.logerr(f" angle --> {self.time*TO_DEGREE}")
            # if(int(self.time/(pi))>int((self.time-self.step-0.0001)/(pi))):
            #     rospy.logerr("oeoeoeoeo")
            #     rospy.logerr("oeoeoeoeo")
            #     rospy.logerr("oeoeoeoeo")
            #     rospy.logerr("oeoeoeoeo")
            #     self.setReverseMode(self.reverse)
            #     self.reverse = not self.reverse

        # self.time += self.step
        self.next_hp()

        # send info tf, plan, arrow got hunting point
        self.publish_visualize()    
        self.publish_huntTF()
        self.publish_plan()


    """     calculate new hunting point    """
    def goal_reached(self):

        # calc turtle dist from hunt point
        dist = self.get_dist()
        rospy.logdebug(f"PATH PLAN --> Turtle Dist from target {dist:.2f}")
        self.threshold = 0.1
        if(dist<self.threshold):
            return True
        
        return False
        
    """     calculate distance from turtle to hunting point    """
    def get_dist(self):
        try:
            trans = self.tfBuffer.lookup_transform("base_footprint", 'hunt_point', rospy.Time())
            dist = sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            return dist
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("PATH PLAN --> PROBLEMO WITH TFS ")
            return 1

    """     calculate new hunting point    """
    def next_hp(self):
        # calculate new hunting point
        if(MODE=="CIRCLE"):
            pt = self.path_circle(self.time)
        
        if(MODE=="LINE"):
            pt = self.path_updown(self.time)

        #  update hunting point
        self.huntPT = pt 


    def get_pose(self):
        euler=[0,0,self.dir]
        pos = self.huntPT
        quad = quaternion_from_euler(euler[0],euler[1],euler[2])
        ori = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
        pose = Pose(position = pos, orientation=ori)
        return pose

    def get_tf(self):
        euler=[0,0,self.dir]
        pos_temp = self.huntPT
        pos = Vector3(x = pos_temp.x, y = pos_temp.y, z = pos_temp.z)
        quad = quaternion_from_euler(euler[0],euler[1],euler[2])
        ori = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
        hp_tf = Transform(translation = pos, rotation=ori)
        return hp_tf


    """
    equation of circle path, based on time input
    """    
    def path_circle(self,time):
        self.polar['th'] = time
        x  = (self.polar['r'])*cos(self.polar['th']) + self.center['x']
        y  = (self.polar['r'])*sin(self.polar['th']) + self.center['y']
        z  = 0
        point = Point(x=x,y=y,z=z)
        self.dir = self.polar['th']+self.offsetDIR
        return point

    """
    equation of circle path, based on time input
    """    
    def path_updown(self,time):
        self.polar['th'] = time
        x  = self.center['x'] + (self.rangeUPDOWN)*cos(self.polar['th'])
        y  = self.center['y']
        z  = 0
        point = Point(x=x,y=y,z=z)
        self.dir = 0
        return point

    """
    create arrow marker object
    """
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

    """
    publish visualization arrow of hunting point
    """
    def publish_visualize(self):
        # add arrown at hunt point
        self.add_marker(    type=self.markerType['ARROW'],
                            size=ARROW_SIZE,
                            color=COLOR_Blue,
                            pose=self.get_pose(),
                            id=0)

        # Publish the MarkerArray
        self.publisherVis.publish(self.markerArray)

    """
    publish the tf of the hunting point
    """
    def publish_huntTF(self):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "hunt_point"
        t.transform = self.get_tf()
        br.sendTransform(t)  
        
    """
    publish the global path plan
    """
    def publish_plan(self):
        self.msg_path.header.frame_id = "world"
        self.msg_path.header.stamp = rospy.Time.now()
        self.publishPath.publish(self.msg_path) 

    """
    set reverse mode on/off at navigation node
    """
    def setReverseMode(self,mode):
        rospy.wait_for_service('/Navigate/ReverseMode')
        try:
            call_srv = rospy.ServiceProxy('/Navigate/ReverseMode', SetBool)
            resp1 = call_srv(mode)
            rospy.logdebug("PATH PLAN --> navigation said " + resp1.message)
        except rospy.ServiceException as e:
            rospy.logdebug("PATH PLAN --> Service call failed: %s"%e)


""" INIT    """
def start():

    rospy.init_node('Path_Planning', log_level=rospy.DEBUG)
    rate = rospy.Rate(5) # publish freacuancy 
    hunt_point = HUNT_POINT()

    while not rospy.is_shutdown():
        rospy.logdebug("PATH PLAN--> --> loopa")
        hunt_point.update()
        rate.sleep()

"""  MAIN    """
if __name__ == '__main__':
    try:
        start() 
    except rospy.ROSInterruptException:
        pass
