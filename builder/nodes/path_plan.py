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
from builder.srv import PathPlanInfo

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



class HUNT_POINT():


    def __init__(self):

        self.center = {"x":0.5,"y":0} # center
        self.dir = 0 
        self.step_size = 2
        self.step = 2*pi/self.step_size
        self.offsetDIR = pi/2
        self.reverse = True
        self.time = 0 
        self.stop = False

        self.mode_count = 0 

        # SET MODE 
        self.MODE = "CIRCLE"
        # self.MODE = "LINE"
        # self.MODE = "REFILL"
        # self.MODE = "RESET"
        


        if(self.MODE == "CIRCLE"):
            self.step_size = 10
            self.step = 2*pi/self.step_size
            self.center = {"x":0.5,"y":0} # offset of circle center
            self.range = 0.3


        if(self.MODE == "LINE"):
            self.step_size = 2
            self.step = 2*pi/self.step_size

            # line 1 
            self.center = {"x":1.1,"y":0.15} # offset of LINE center
            self.range = 0.3


            # # line 1 cooling
            # self.center = {"x":1.2,"y":0.15} # offset of LINE center
            # self.range = 0.3

            # # # line 2 
            self.center = {"x":1.3,"y":0.25} # offset of LINE center
            self.range = 0.3

            # # line 2 cooling
            # self.center = {"x":1.5,"y":0.25} # offset of LINE center
            # self.range = 0.3



        self.huntPT = Point(x=0,y=0,z=0)


        self.publisherVis  = rospy.Publisher("hunt_point_vis" , MarkerArray,queue_size=10)
        self.publishPath   = rospy.Publisher("hunt_point_path", Path,queue_size=10)
        self.msg_path = Path()
        

        self.markerArray = MarkerArray()
        self.markerArray.markers.append(Marker())
        self.markerType = {"ARROW":0}


        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)


        rospy.Service('/path_plan/set_plan', PathPlanInfo, self.setPlan)
        rospy.Service('/path_plan/stop'   , SetBool, self.activatePlan)

        # draw path plan 
        self.drawPLAN()

        # self.time = random.random()*2*pi

        # send info tf, plan, arrow got hunting point
        self.publish_visualize()    
        self.publish_huntTF()

        self.setReverseMode(self.reverse)


    """     main loop    """
    def update(self):
        rospy.logdebug("PATH PLAN--> update")
        rospy.logdebug(f"PATH PLAN--> TIME {self.time}")
        rospy.logdebug(f"PATH PLAN--> STOP {self.stop}")
        rospy.logdebug(f"PATH PLAN--> MODE {self.MODE}")

        if(self.goal_reached() and self.stop==False):
            rospy.logdebug("PATH PLAN--> Goal Reached")
            self.time += self.step

            # move to next step after one cycle 
            if (self.time>=2*pi):
                rospy.loginfo("MOVING TO NEXT STATE")
                rospy.sleep(2)
                rospy.loginfo(f"STATE NUM --> {self.mode_count}")
                # self.stop = True

                
                # if(self.mode_count==1):
                #     self.center = {"x":1.1,"y":0.15} # offset of LINE center
                #     self.range = 0.3
                # elif(self.mode_count==2):
                #     self.MODE = "RESET"
                #     self.reverse = True
                # elif(self.mode_count==3):
                #     self.MODE = "REFILL"
                #     self.reverse = True
                # elif(self.mode_count==4):
                #     self.MODE = "RESET"
                #     self.reverse = True
                # elif(self.mode_count==5):
                #     self.MODE = "LINE"
                #     self.center = {"x":1.1,"y":0.25} # offset of LINE center
                #     self.range = 0.3
                #     self.time = 0 
                # elif(self.mode_count==6):
                #     self.MODE = "RESET"
                #     self.mode_count=0
                #     self.reverse = True
                    
                self.time = 0 
                # self.mode_count = self.mode_count + 1 # next step
            
            if(self.MODE == "LINE"):
                self.reverse = not self.reverse
                self.setReverseMode(self.reverse)


        self.next_hp()


        # send info tf, plan, arrow got hunting point
        self.publish_visualize()    
        self.publish_huntTF()
        self.publish_plan()


    """     calculate new hunting point    """
    def goal_reached(self):

        # calc turtle dist from hunt point
        dist = self.get_dist()
        # rospy.logdebug(f"PATH PLAN --> Turtle Dist from target {dist:.2f}")
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
        if(self.MODE=="CIRCLE"):
            pt = self.path_circle(self.time)
        
        if(self.MODE=="LINE"):
            pt = self.path_updown(self.time)
        
        
        if(self.MODE=="REFILL"):
            pt = Point(x=0.45,y=0,z=0)
            self.reverse = False
            self.setReverseMode(self.reverse) 

        if(self.MODE=="RESET"):
            pt = Point(x=1.0,y=0,z=0)
            self.reverse = True
            self.setReverseMode(self.reverse) 

        #  update hunting point
        self.huntPT = pt 


    def drawPLAN(self):
        # reset time
        self.time = 0 
        self.msg_path.poses = []
        while(self.time<2*pi+0.01):
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
        
        x  = (self.range)*cos(time) + self.center['x']
        y  = (self.range)*sin(time) + self.center['y']
        z  = 0
        point = Point(x=x,y=y,z=z)
        self.dir = time+self.offsetDIR
        return point

    """
    equation of circle path, based on time input
    """    
    def path_updown(self,time):
        
        x  = self.center['x'] + (self.range)*cos(time)
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


    ############################  SERVICES ############################
    """
    set reverse mode on/off at navigation node
    """
    def setReverseMode(self,mode):
        rospy.wait_for_service('/navigate/reverse_mode')
        try:
            call_srv = rospy.ServiceProxy('/navigate/reverse_mode', SetBool)
            resp1 = call_srv(mode)
            rospy.logdebug("PATH PLAN --> navigation said " + resp1.message)
        except rospy.ServiceException as e:
            rospy.logdebug("PATH PLAN --> Service call failed: %s"%e)

    """
    Service for activation of path planing
    call service True  --> pause  next goals
    call service False --> resume next goals
    """
    def activatePlan(self,SetBoolRequest):
        if(SetBoolRequest.data):
            self.stop = True
            msg = "PATH PLAN--> OFF"
        else:
            self.stop = False
            msg = "PATH PLAN--> ON"

        rospy.logdebug(msg)
        return SetBoolResponse(success = True,message =msg)
        
    """
    Service for setting information for path planing
    offset of center 
    step size 
    range

    """
    def setPlan(self,srv_msg):
        self.center['x'] = srv_msg.centerX
        self.center['y'] = srv_msg.centerY
        self.range = srv_msg.range
        self.step = srv_msg.step
        self.step_size = srv_msg.step_size
        self.MODE = srv_msg.mode
        if(self.MODE == "LINE"):
            self.reverse = False
            self.setReverseMode(self.reverse)
        self.drawPLAN()
        rospy.logdebug("PATH PLAN--> --> upadate plan params")
        return "Updating planing parameters"
        

""" INIT    """
def start():

    rospy.init_node('Path_Planning', log_level=rospy.DEBUG)
    rate = rospy.Rate(20) # publish freacuancy 
    hunt_point = HUNT_POINT()
    # main loop
    while not rospy.is_shutdown():
        if(hunt_point.goal_reached()):
            hunt_point.update()
        hunt_point.publish_huntTF()
        rate.sleep()

"""  MAIN    """
if __name__ == '__main__':
    try:
        start() 
    except rospy.ROSInterruptException:
        pass
