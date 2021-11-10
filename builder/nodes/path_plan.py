#!/usr/bin/env python

import random
import rospy
from std_srvs import srv
from tf.transformations import quaternion_from_euler
from math import pi,cos,sin
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3,Pose,Point, Quaternion,Transform,TransformStamped,PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from std_srvs.srv import SetBool,SetBoolResponse
from builder.srv import PathPlanInfo

import actionlib
import builder.msg
import tf2_ros
import actionlib


ARROW_SIZE  = Vector3(x=0.3,y=0.05,z=0.05)    # x  length   y,z size
COLOR_Blue     = ColorRGBA(r=0.0,g=0.0,b=1.0,a=1.0) # Blue 
COLOR_Red      = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0) # Red
COLOR_Yellow   = ColorRGBA(r=1.0,g=1.0,b=0.0,a=1.0) # Yellow
COLOR_Green    = ColorRGBA(r=0.0,g=1.0,b=0.0,a=1.0) # Green
COLOR_Purple   = ColorRGBA(r=1.0,g=0.0,b=1.0,a=1.0) # Green

TO_DEGREE = 57.2958


class PLAN():
    def __init__(self):
        self.center = {"x":0.5,"y":0} 
        self.step_size = 20
        self.step = 2*pi/self.step_size
        self.range = 0.5
        self.reverse = True
        self.MODE = "CIRCLE"

        # self.center = {"x":2*random.random(),"y":2*random.random()} 
        # self.step_size = 2
        # self.step = 2*pi/self.step_size
        # self.range = 0.5
        # self.reverse = True
        # self.MODE = "POINT"


    def copy_plan_from_msg(self,msg):
        self.step_size   = msg.step_size
        self.center['x'] = msg.centerX
        self.center['y'] = msg.centerY
        self.reverse = msg.reverse
        self.range   = msg.range
        self.step    = msg.step
        self.MODE    = msg.mode

    def get_goal(self):
        return builder.msg.huntGoal(reverseMD = self.reverse, debugMD = False)

class PLANNER():


    def __init__(self):

        self.center = {"x":0.5,"y":0} # center
        self.dir = 0 
        self.offsetDIR = pi/2
        self.reverse = True
        self.time = 0 
        self.stop = False
        self.active = True

        self.plan = PLAN()

        # SET MODE 
        # self.MODE = "CIRCLE"
        # self.MODE = "LINE"
        # self.MODE = "POINT"
        


        # if(self.MODE == "CIRCLE"):
        #     self.step_size = 10
        #     self.step = 2*pi/self.step_size
        #     self.center = {"x":0.5,"y":0} # offset of circle center
        #     self.range = 0.3


        # if(self.MODE == "LINE"):
        #     self.step_size = 2
        #     self.step = 2*pi/self.step_size

        #     # line 1 
        #     self.center = {"x":1.1,"y":0.15} # offset of LINE center
        #     self.range = 0.3



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


        # send info tf, plan, arrow got hunting point
        self.publish_visualize()    
        self.publish_huntTF()

        # Creates the SimpleActionClient and wait for server to come up
        self.hunting_action = actionlib.SimpleActionClient('action_hunt', builder.msg.huntAction)
        self.hunting_action.wait_for_server()


    """
         UPDATE
    call navigation hunting action with new point
    """
    def update(self):
        rospy.logdebug("PATH PLAN--> update")
        rospy.logdebug(f"PATH PLAN--> MODE {self.plan.MODE}")
        rospy.logdebug(f"PATH PLAN--> TIME {self.time}")
        rospy.logdebug(f"PATH PLAN--> POINT {self.plan.center}")

        if(self.goal_reached()):
        # if(True):
            

            # disable planer if path is executed
            if (self.time>=2*pi):
                rospy.logdebug("PATH PLAN--> FINISH")
                self.active = False
            
            self.next_hp()


            # send info tf, plan, arrow got hunting point
            self.publish_visualize()    
            self.publish_huntTF()
            self.publish_plan()

            # Creates a goal to send to the action server.
            goal = self.plan.get_goal()
            self.hunting_action.send_goal(goal) # Sends the goal to the action server.

            # update time
            self.time += self.plan.step


    """     calculate new hunting point    """
    def goal_reached(self):

        # rospy.logdebug(f"PATH PLAN --> robot Dist from target {dist:.2f}")
        # rospy.logdebug(f"PATH PLAN --> robot Dir  from target {dir:.2f}")
        
        rospy.logerr(f"result ---> {self.hunting_action.get_result()}")
        rospy.logerr(f"state ---> {self.hunting_action.get_state()}")
        goalID = self.hunting_action.get_state()

        if(goalID==0):
            rospy.logerr("HUNTING ACTION STATUS --> PENDING")
        elif(goalID==1):
            rospy.logerr("HUNTING ACTION STATUS --> ACTIVE")
            return False
        elif(goalID==2):
            rospy.logerr("HUNTING ACTION STATUS --> PREEMPTED")
        elif(goalID==3):
            rospy.logerr("HUNTING ACTION STATUS --> SUCCEEDED")
            return True
        elif(goalID==4):
            rospy.logerr("HUNTING ACTION STATUS --> ABORTED")
        elif(goalID==5):
            rospy.logerr("HUNTING ACTION STATUS --> REJECTED")
        elif(goalID==6):
            rospy.logerr("HUNTING ACTION STATUS --> PREEMPTING")
        elif(goalID==7):
            rospy.logerr("HUNTING ACTION STATUS --> RECALLING")
        elif(goalID==8):
            rospy.logerr("HUNTING ACTION STATUS --> RECALLED")
        elif(goalID==9):
            rospy.logerr("HUNTING ACTION STATUS --> LOST")
            return True
        else:
            rospy.logerr("ERROR")
            rospy.logerr("ERROR")
            rospy.logerr("ERROR")
            rospy.logerr("ERROR")
            rospy.logerr("ERROR")
            


        # if(self.hunting_action.get_result()):
        #     rospy.logdebug("PATH PLAN--> Goal Reached")
        #     return True
        return False
        

    """     calculate new hunting point    """
    def next_hp(self):
        # calculate new hunting point
        if(self.plan.MODE=="CIRCLE"):
            pt = self.path_circle(self.time)
        
        if(self.plan.MODE=="LINE"):
            pt = self.path_updown(self.time)
        
        
        if(self.plan.MODE =="POINT"):
            pt = Point(x=self.plan.center['x'],y=self.plan.center['y'],z=0)

        #  update hunting point
        self.huntPT = pt 


    def drawPLAN(self):
        # reset time
        self.time = 0 
        self.msg_path.poses = []
        while(self.time<2*pi+0.01):
            self.time += self.plan.step
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
        
        x  = (self.plan.range)*cos(time) + self.plan.center['x']
        y  = (self.plan.range)*sin(time) + self.plan.center['y']
        z  = 0
        point = Point(x=x,y=y,z=z)
        self.dir = time+self.offsetDIR
        return point

    """
    equation of circle path, based on time input
    """    
    def path_updown(self,time):
        
        x  = self.plan.center['x'] + (self.plan.range)*cos(time)
        y  = self.plan.center['y']
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
        self.plan.copy_plan_from_msg(srv_msg)
        self.drawPLAN()
        self.active = True
        rospy.loginfo("PATH PLAN--> GOT NEW PLAN")
        return "Updating with new planing parameters"
        

""" INIT    """
def start_planner():

    rospy.init_node('path_planning', log_level=rospy.DEBUG)
    # rate = rospy.Rate(20) # publish freacuancy 
    rate = rospy.Rate(20) # publish freacuancy 
    planner = PLANNER()
    # main loop
    while not rospy.is_shutdown():
        
        # start new plan
        if(planner.active):
            planner.update()
        
        # publish hunting tf point always (and fast)
        planner.publish_huntTF()
        rate.sleep()

"""  MAIN    """
if __name__ == '__main__':
    try:
        start_planner() 
    except rospy.ROSInterruptException:
        pass
