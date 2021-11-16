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



ARROW_SIZE  = Vector3(x=0.3  ,y=0.05,z=0.05)    # x  length   y,z size
SPHERE_SIZE = Vector3(x=0.05 ,y=0.05,z=0.05)    # x  length   y,z size
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
        self.direction = 0 
        self.printMD = False
        self.debug_mode = rospy.get_param("/debug_mode",default=False)

    def copy_plan_from_msg(self,msg):
        self.step_size   = msg.step_size
        self.init_time    = msg.init_time
        self.step = 2*pi/self.step_size
        self.center['x'] = msg.centerX
        self.center['y'] = msg.centerY
        self.reverse = msg.reverse
        self.range   = msg.range
        self.MODE    = msg.mode
        self.direction = msg.direction
        self.printMD = msg.printMD

        
        
        if(self.MODE == "LINE"): # Do only start and finish point of the line, NO STEPS FOR NOW
            self.step_size   = 2 
            self.step = 2*pi/self.step_size

    def get_goal(self,time):
        if(self.MODE=="LINE"):
            if(time>0):
                self.reverse = False
            else:
                self.reverse = True

        return builder.msg.huntGoal(reverseMD = self.reverse, debugMD = self.debug_mode)

    def get_direction(self,time):
        if(self.MODE=="LINE" and time>0): # this is for cos 
            return self.direction + pi
        return self.direction


class PLANNER():


    def __init__(self):

        self.dir = 0 
        self.offsetDIR = pi/2
        self.time = 0 
        self.stop = False
        self.success = False

        self.plan = PLAN()

        self.huntPT = Point(x=0,y=0,z=0)


        self.publisherVis  = rospy.Publisher("hunt_point_vis" , MarkerArray,queue_size=10)
        self.publishPath   = rospy.Publisher("hunt_point_path", Path,queue_size=10)
        self.msg_path = Path()
        

        self.markerArray = MarkerArray()
        self.markerType = {"ARROW":0,"SPHERE":2}
        self.markerArray.markers.append(Marker())

        # create hunting action 
        self._feedback = builder.msg.PathPlanInfoFeedback()
        self._result   = builder.msg.PathPlanInfoResult()
        self._action = actionlib.SimpleActionServer("action_planner", builder.msg.PathPlanInfoAction, execute_cb=self.execute_action, auto_start = False)
        self._action.start()


        # Creates the SimpleActionClient and wait for server to come up
        self.hunting_action = actionlib.SimpleActionClient('action_hunt', builder.msg.huntAction)
        self.hunting_action.wait_for_server()
    
    """
    get a new plan to follow
    """
    def execute_action(self,goal):
        rospy.loginfo("")
        rospy.loginfo("")
        rospy.loginfo("PATH PLAN --> GOT NEW PLAN")

        self.plan.copy_plan_from_msg(goal)
        self.drawPLAN()
        self.time = goal.init_time
        self.publish_visualize()    
        self.publish_huntTF()
        self.success = False
        rate = rospy.Rate(20) # publish freacuancy 
       
        # start executing the action
        while(not self.success):

            # check that preempt has not been requested by the client
            if self._action.is_preempt_requested():
                rospy.logerr("PATH PLAN --> PLANNER ACTION CANCEL" )
                self._action.set_preempted()
                break
            
            # update & publish the feedback
            self._feedback.time = self.time
            self._action.publish_feedback(self._feedback)


        
            
            # publish hunting tf point always (and fast)
            self.publish_huntTF()

            # play the plan
            self.update()

            rate.sleep()

            if(self.success):
                self._result.success = self.success
                self._action.set_succeeded(result = self._result)
                rospy.loginfo(f"PATH PLAN --> PLANNER ACTION SUCCESS {self.success}")


    """
         UPDATE
    call navigation hunting action with new point
    """
    def update(self):
        
        if(self.goal_reached()):


            # disable planer if path is executed
            if (self.time>=2*pi):
                # rospy.loginfo("PATH PLAN --> FINISH")
                self.success = True
            
            else:
                rospy.loginfo(f"PATH PLAN --> MODE  {self.plan.MODE}")
                rospy.loginfo(f"PATH PLAN --> REVE  {self.plan.reverse}")
                rospy.loginfo(f"PATH PLAN --> TIME  {self.time}")
                rospy.loginfo(f"PATH PLAN --> POINT {self.huntPT.x:.2f},{self.huntPT.y:.2f}")
                rospy.loginfo(f"PATH PLAN --> DIRE  {self.dir*TO_DEGREE:.2f}")
                
                self.next_hp()


                # send info tf, plan, arrow got hunting point
                self.publish_visualize()    
                self.publish_huntTF()
                self.publish_plan()

                # Creates a goal to send to the action server.
                goal = self.plan.get_goal(self.time)
                self.hunting_action.send_goal(goal) # Sends the goal to the action server.

            # update time
            self.time += self.plan.step


    """     calculate new hunting point    """
    def goal_reached(self):

        goalID = self.hunting_action.get_state()
        if(goalID==1):
            # rospy.loginfo("HUNTING ACTION STATUS --> ACTIVE")
            return False
        if(goalID==2):
            # rospy.loginfo("HUNTING ACTION STATUS --> PREEMPTED")
            self.time -= self.plan.step # repeat hunting point
            return True
        elif(goalID==3):
            # rospy.loginfo("HUNTING ACTION STATUS --> SUCCEEDED")
            return True
        elif(goalID==4):
            # rospy.loginfo("HUNTING ACTION STATUS --> ABORTED")
            return True
        elif(goalID==9):
            # rospy.loginfo("HUNTING ACTION STATUS --> LOST")
            return True
        else:
            rospy.logerr(f"HUNTING ACTION STATUS --> ERROR {goalID}")
            rospy.logerr(f"HUNTING ACTION STATUS --> ERROR {goalID}")

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
            self.dir = self.plan.direction 

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
        self.dir = self.plan.get_direction(self.time)
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
        

        
        if(id == 0 ):
            self.markerArray.markers[id] = marker    
        else:
            self.markerArray.markers.append(marker)


    """
    publish visualization arrow of hunting point
    """
    def publish_visualize(self):

        if(len(self.markerArray.markers)>50):
            self.markerArray.markers = []
            self.markerArray.markers.append(Marker)

        # add arrow at hunt point
        col = COLOR_Blue
        if(self.plan.reverse):
            col = COLOR_Yellow
        self.add_marker(    type=self.markerType['ARROW'],
                            size=ARROW_SIZE,
                            color=col,
                            pose=self.get_pose(),
                            id=0)

        # keep track
        col = COLOR_Blue
        if(self.plan.printMD):
            col = COLOR_Red
        self.add_marker(    type=self.markerType['SPHERE'],
                            size=SPHERE_SIZE,
                            color=col,
                            pose=self.get_pose(),
                            id=len(self.markerArray.markers))

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

        

"""  MAIN    """
if __name__ == '__main__':
    try:
        # rospy.init_node('path_planning', log_level=rospy.DEBUG)
        rospy.init_node('path_planning', log_level=rospy.INFO)
        planner = PLANNER()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("PATH PLAN --> DEAD")
