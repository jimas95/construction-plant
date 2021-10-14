#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse,TriggerRequest,Empty,EmptyRequest,EmptyResponse
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from moveit_commander.conversions import pose_to_list
from arm_move.srv import step_srv,step_srvResponse,step_srvRequest
import tf2_ros

""" NODE RobotPX, robot: px100

Services : 
        '/px100/get_joint_states' --> type:Trigger | return current joint state
        '/px100/get_eef_pose'     --> type:Trigger | return end effector position
        '/px100/pathPlan'         --> type:Empty   | makes a pre set plan based on cartesian pathplaning
        '/px100/stop'             --> type:Empty   | stop the infinete loop of follow service(if active)
        '/px100/saveWaypoints'    --> type:Empty   | saves waypoints into parameter server namespace /waypoints/points/pt1-N
        '/px100/clearWaypoints'   --> type:Empty,  | clear waypoint list, delete parameter server points
        '/px100/follow'           --> type:SetBool | starts from current position and executes a path plan for all waypoints and finish at the home position, if Bool--> True itterated infinetly throught the waypoints
        '/px100/reset'            --> type:SetBool | resets world obstacles, if bool --> True removes all stored waypoints
        '/px100/step'             --> type:step_srv| executes (if possible) a path plan from current position to set position args --> Pose : goal position , bool : at end state have gripper open/close

RobotPX does not have any Publishers or Subscribers but it does communicate with the rest of the world using the Moveti pgk

"""




class RobotPX():
    """
    Init of class robot px100
    initialize/setup robot control 
    create RobotCommander,PlanningSceneInterface,MoveGroupCommander(robot & gripper)
    create services 
    create my world
    load waypoints
    """
    def __init__(self):
        super(RobotPX, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('RobotPX',anonymous=True,log_level=rospy.DEBUG)
        rospy.init_node('RobotPX',anonymous=True)
        robot_name = "px100"
        self.robot_name = robot_name
        self.dof = 4

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "interbotix_arm"
        
        group = moveit_commander.MoveGroupCommander(group_name)
        group_gripper_name = "interbotix_gripper"
        group_gripper = moveit_commander.MoveGroupCommander(group_gripper_name)
        
        display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        ## Getting Basic Information
        planning_frame = group.get_planning_frame()
        print( "============ Reference frame: %s" % planning_frame)

        eef_link = group.get_end_effector_link()
        print( "============ End effector: %s" % eef_link)

        group_names = robot.get_group_names()
        print( "============ Robot Groups:", robot.get_group_names())

        print( "============ print(ing robot state")
        print( robot.get_current_state())
        print( "")


        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.group_gripper = group_gripper
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.loopa = False
        #create services 
        self.srv_joint_state  = rospy.Service('get_joint_states', Trigger, self.srvf_joint_state)
        self.srv_eef_position = rospy.Service('get_eef_pose', Trigger, self.srvf_eef_position)
        self.srv_pathPlan = rospy.Service('pathPlan', Empty, self.srvf_pathPlan)
        self.srv_reset = rospy.Service('reset', SetBool, self.srvf_reset)
        self.srv_Stop = rospy.Service('stop', Empty, self.srvf_Stop)
        self.srv_saveWaypoints = rospy.Service('saveWaypoints', Empty, self.srvf_saveWaypoints)
        self.srv_clearWaypoints = rospy.Service('clearWaypoints', Empty, self.srvf_clearWaypoints)
        self.srv_follow = rospy.Service('follow', SetBool, self.srvf_follow)
        self.srv_step = rospy.Service('step', step_srv, self.srvf_step)

        #create my scene 
        self.create_my_scene()
        msg = SetBoolRequest()
        msg.data = False
        self.srvf_reset(msg)

        #load waypoints
        data = rospy.get_param("/points")
        temp_list = list(data.items())
        self.waypoints = []
        for item in temp_list:
            point = item[1]
            waypoint = step_srvRequest()
            waypoint.pose.position.x = point[0]
            waypoint.pose.position.y = point[1]
            waypoint.pose.position.z = point[2]
            waypoint.pose.orientation.x = point[3]
            waypoint.pose.orientation.y = point[4]
            waypoint.pose.orientation.z = point[5]
            waypoint.pose.orientation.w = point[6]

            if(point[7]==1):
                waypoint.gripper =True
            else:
                waypoint.gripper =False

            self.waypoints.append(waypoint)
        self.srvf_saveWaypoints(EmptyRequest)
    """
    main update loop
    does not do much only debuging mesagges
    everything is being executed and controled using services
    """
    def update(self):
        rate = rospy.Rate(1) # publish freacuancy 
        while not rospy.is_shutdown():
            rospy.loginfo("Hello!")
            self.print_joint_state()
            # self.get_eef_pose(True)
            # rospy.loginfo(self.group_gripper.get_current_joint_values())
            # self.group.set_named_target("Home")
            # self.group.go(wait=True)
            # self.group.set_named_target("Sleep")
            # self.group.go(wait=True)
            rate.sleep()

    """
    return the current joint values 
    """
    def get_joint_state(self):
        return self.group.get_current_joint_values()
    
    """
    print all the current joint values 
    """
    def print_joint_state(self):
        joint_state = self.group.get_current_joint_values()
        i=0
        for joint in joint_state: 
            rospy.loginfo("joint "+str(i)+ ": " +str(joint*180/pi))
            i+=1
        rospy.loginfo("------")


    """
    return the current eef position based an world frame
    setting input bool to true will print the value
    """
    def get_eef_pose(self,bool_print):
        current_pose = self.group.get_current_pose().pose
        if(bool_print): rospy.loginfo("eef position %s",current_pose)
        return current_pose

# ************************************ functions handling scene objects  *************************************

    """
    create our scene setup 
    2 tables table2 is on table1 each table has its own legs
    """
    def create_my_scene(self):  
        tableHeight = 0.05
        tableSize = 0.5
        table1_position = [-0.1,0,0]
        self.add_table(width=0.25,height=0.6,position=table1_position,name="table1")
        leg_size = 0.025
        self.add_leg([ -0.2,-0.3 +leg_size/2 ,-tableHeight+table1_position[2]],0.25,"leg5")
        self.add_leg([ -0.2,0.3 -leg_size/2,-tableHeight+table1_position[2]],0.25,"leg6")
        

        
        tableHeight = 0.05
        tableSize = 0.5
        table2_position = [0,0,-0.25]
        self.add_table(width=0.5,height=1,position=table2_position,name="table2")
        leg_size = 0.025
        self.add_leg([-tableSize/2 +leg_size/2,-tableSize +leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg1")
        self.add_leg([-tableSize/2 +leg_size/2, tableSize -leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg2")
        self.add_leg([ tableSize/2 -leg_size/2,-tableSize +leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg3")
        self.add_leg([ tableSize/2 -leg_size/2, tableSize -leg_size/2 ,-tableHeight+table2_position[2]],0.5,"leg4")


    """
    add a "table" at out scene 
    """    
    def add_table(self,width,height,position,name):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2] - 0.05/2
        self.scene.add_box(name, box_pose, size=(width, height, 0.05))

        rospy.loginfo("ADDING OBJECT --> "+ name)
        if(not self.wait_for_state_update(objName = name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ name)

    """
    add leg(streched box) object
    """
    def add_leg(self,position,length,name):
        leg_size = 0.025
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2] - length/2
        self.scene.add_box(name, box_pose, size=(leg_size, leg_size, length))

        rospy.loginfo("ADDING OBJECT --> "+ name)
        if(not self.wait_for_state_update(objName = name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ name)

    """
    add a box that will be our grasping object at the scene
    """
    def add_graspObject(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.2
        box_pose.pose.position.y = -0.12
        box_pose.pose.position.z = -0.25 + 0.1
        self.box_name = "graspObject"
        self.scene.add_box(self.box_name, box_pose, size=(0.025, 0.025, 0.1))

        if(not self.wait_for_state_update(objName = self.box_name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ self.box_name)

    """
    add realSense sense object(box) as an obstacle
    """
    def add_obstacle(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.2
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = -0.25 + 0.09
        self.box_name = "obstacle"
        self.scene.add_box(self.box_name, box_pose, size=(0.14, 0.05,0.09))

        if(not self.wait_for_state_update(objName = self.box_name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ self.box_name)
       
    """
    wait function to be sure that an object has been added to our scene before continue
    """
    def wait_for_state_update(self,objName, box_is_known=False, box_is_attached=False, timeout=5):
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([objName])
            is_attached = len(attached_objects.keys()) > 0
            # rospy.logerr("still trying to get the object "+ objName)
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = objName in self.scene.get_known_object_names()
            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    """
    attach the graspObject to eef robot
    """
    def attach_box(self):
        grasping_group = 'interbotix_gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "graspObject", touch_links=touch_links)
        return self.wait_for_state_update(objName="graspObject",box_is_attached=True, box_is_known=False, timeout=4)

    """
    detach object named obj_name from eeo
    """
    def detach_box(self,obj_name):
        self.scene.remove_attached_object(self.eef_link, name=obj_name)
        return self.wait_for_state_update(obj_name,box_is_known=True, box_is_attached=False, timeout=4)

    """
    remove obj_name from scence
    """
    def remove_obj(self,obj_name):
        self.scene.remove_world_object(obj_name)
        return self.wait_for_state_update(obj_name,box_is_attached=False, box_is_known=False, timeout=4)

    """
    return a boolean if an obj_name exists in already in the scene
    """
    def object_exists(self,obj_name):
        is_known = obj_name in self.scene.get_known_object_names()
        return is_known

    """
    find a series of points(plan) for predifined goal state 
    using group.compute_cartesian_path
    """
    def plan_cartesian_path(self):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x =  0.12
        wpose.position.y =  0.0
        wpose.position.z = -0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x =  0.16
        wpose.position.y =  0.0
        wpose.position.z =  0.044
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x =  0.19
        wpose.position.y =  0.0
        wpose.position.z =  0.15
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.005,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    """display a trajectory path virtualy, (does not move the robot)
    """
    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

# ************************************** SERVICES ***************************************

    """
    Service 
    returns a string msg with all the current joint values
    """
    def srvf_joint_state(self,TriggerRequest):
        joint_state = self.get_joint_state()
        data = ""
        i=0
        for joint in joint_state: 
            data+= "joint " + str(i) + ": " + str(joint*180/pi) + "\n"
            i+=1
        msg = TriggerResponse()
        msg.message = str(joint_state)
        msg.success = True
        return msg

    """
    Service 
    returns the curren eef postion relative to the world frame
    """
    def srvf_eef_position(self,TriggerRequest):
        msg = TriggerResponse()
        current_pose = self.group.get_current_pose().pose
        msg.message = "position: " + str(current_pose.position) + " orientation: " + str(current_pose.orientation)
        msg.success = True
        return msg

    """Service computes a pre defined cartesian path
    """
    def srvf_pathPlan(self,EmptyRequest):
        rospy.loginfo("Finding path")
        self.plan, fraction = self.plan_cartesian_path()
        rospy.loginfo("path FOUND")
        self.display_trajectory(self.plan)
        return EmptyResponse()

    """Service step
    has an inpute of step_srvRequest (pose,bool open/close gripper)
    if path found exucute and store it in parameter server under the /waypoint namespace
    """
    def srvf_step(self,step_srvRequest):
        msg = step_srvResponse()

        self.group.set_pose_target(step_srvRequest.pose)
        plan = self.group.plan()
        msg.ErrorCode = plan[3]

        if(not plan[0]): 
            rospy.logerr("ERROR path plan NOT FOUND")

            return msg

        self.group.execute(plan[1], wait=True)
        if(step_srvRequest.gripper):
            self.group_gripper.set_named_target("Home")
            self.group_gripper.go(wait=True)
        else:
            self.group_gripper.set_named_target("Open")
            self.group_gripper.go(wait=True)

        self.waypoints.append(step_srvRequest)
        self.srvf_saveWaypoints(EmptyRequest)
        return msg


    """
    Service stop 
    it will make the follow service iteration infinete loop to stop (if it weas active)
    """
    def srvf_Stop(self,EmptyRequest):
        self.loopa = False
        return EmptyResponse()


    """ 
    SERVICE follow
    iterates all the waypoints and executes them one by one 
    """
    def srvf_follow(self,SetBoolRequest):
        if(SetBoolRequest.data):
            self.loopa = True
        while(self.loopa):
            rospy.loginfo("TI FASI")
            for point in self.waypoints:
                # execute whole planning
                rospy.loginfo(point)
                self.group.set_pose_target(point.pose)
                plan = self.group.plan()
                if(not plan[0]): 
                    rospy.logerr("ERROR path plan NOT FOUND ?")
                    return EmptyResponse()

                self.group.execute(plan[1], wait=True)

                # set gripper on/off
                if(point.gripper):
                    joint_goal = self.group_gripper.get_current_joint_values()
                    # joint_goal[0] = 0.0165
                    # joint_goal[1] = -0.0165
                    joint_goal[0] = 0.022
                    joint_goal[1] = -0.022
                    self.group_gripper.set_named_target("Home")
                    # self.group_gripper.go(joint_goal,wait=True)
                    self.group_gripper.go(wait=True)
                    self.attach_box()
                else:
                    joint_goal = self.group_gripper.get_current_joint_values()
                    joint_goal[0] = 0.035
                    joint_goal[1] = -0.035
                    self.group_gripper.set_named_target("Open")
                    # self.group_gripper.go(joint_goal,wait=True)
                    self.group_gripper.go(wait=True)
                    self.detach_box("graspObject")

                
            # go to home position , reset world
            self.group.set_named_target("Home")
            self.group.go(wait=True)
            msg = SetBoolRequest
            msg.data = False
            self.srvf_reset(msg)

        #return msg        
        msg = SetBoolResponse()
        msg.message = " "
        msg.success = True
        return msg


    """SERVICE reset
    Reset scene, delete/create obstacle,grasping obstacle set home robot position
    """
    def srvf_reset(self,SetBoolRequest):
        res = self.object_exists("obstacle")
        if(res):
            self.remove_obj("obstacle")
        self.add_obstacle()

        res = self.object_exists("graspObject")
        if(res):
            self.remove_obj("graspObject")
        self.add_graspObject()

        self.group.set_named_target("Home")
        self.group.go()
        msg = SetBoolResponse()
        msg.message = " "
        msg.success = True
        if(SetBoolRequest.data):
            self.srvf_clearWaypoints(EmptyRequest)
            msg.message = "waypoints reset"
        return msg

    """SERVICE saveWaypoints
    update points in parameters server based 
    on waypoints
    """
    def srvf_saveWaypoints(self,EmptyRequest):
        finalList = []
        i=0
        for item in self.waypoints:
            tempList = []
            tempList.append(item.pose.position.x)
            tempList.append(item.pose.position.y)
            tempList.append(item.pose.position.z)
            tempList.append(item.pose.orientation.x)
            tempList.append(item.pose.orientation.y)
            tempList.append(item.pose.orientation.z)
            tempList.append(item.pose.orientation.w)
            if(item.gripper):
                tempList.append(1)
            else:
                tempList.append(0)
            finalList.append(tempList)

            rospy.set_param("/waypoints/points/pt"+str(i),tempList)
            i+=1
        return EmptyResponse()

    """SERVICE clearWaypoints
    Reset waypoints list and parameters
    """
    def srvf_clearWaypoints(self,EmptyRequest):
        self.waypoints = []
        rospy.delete_param("/waypoints/")
        # self.srvf_saveWaypoints(EmptyRequest)
        return EmptyResponse()

"""
creates object of class and calls main loop 
"""
if __name__ == '__main__':
    try:
        robot = RobotPX()
        robot.update()
    except rospy.ROSInterruptException:
        pass