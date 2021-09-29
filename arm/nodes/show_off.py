#!/usr/bin/env python

from os import name
import sys
import random
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse,TriggerRequest,Empty,EmptyRequest,EmptyResponse
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from moveit_commander.conversions import pose_to_list, list_to_pose
from arm_move.srv import step_srv,step_srvResponse,step_srvRequest
import tf2_ros
# from Myscene import MySceneMoveIt
# from arm.import_me_if_you_can import say_it_works
from arm.Myscene import MySceneMoveIt


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
        scene_ = moveit_commander.PlanningSceneInterface()
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

        print("")


        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = MySceneMoveIt(scene_,robot,eef_link)
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
        # msg = SetBoolRequest()
        # msg.data = False
        # self.srvf_reset(msg)

        #load waypoints
        self.config_joints = rospy.get_param("/waypoints_joints")
        self.config_eepose = rospy.get_param("/waypoints_eepose")
        

        data = rospy.get_param("/points")
        temp_list = list(data.items())
        self.waypoints = []
        self.waypoints_joints = []
        
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
        # self.srvf_saveWaypoints(EmptyRequest)
    """
    main update loop
    does not do much only debuging mesagges
    everything is being executed and controled using services
    """
    def update(self):
        rate = rospy.Rate(1) # publish freacuancy 
        self.openGR()
        while not rospy.is_shutdown():
            rospy.loginfo("Hello!")

            # self.go_to()
            # self.pick_and_place()
            self.pick_and_place2()
            # self.random_pose()

            # set random postiion of candle
            # pos = [random.random()/2,random.random()/2,0.01]
            # self.scene.set_candle_pos(pos)

            rate.sleep()


    def pick_and_place(self):

        for eef_goal in ["home","home_candle","pre_grab","grab","home_candle","pre_place","place","pre_place"]:        
            pose = list_to_pose(self.config_eepose[eef_goal])
            self.go_to(pose)
            if(eef_goal=="grab"):
                self.closeGR()
            if(eef_goal=="place"):
                self.openGR()

        for eef_goal in ["home_candle","pre_place","place","home_candle","pre_grab","grab","pre_grab","home_candle"]:        
            pose = list_to_pose(self.config_eepose[eef_goal])
            self.go_to(pose)
            if(eef_goal=="grab"):
                self.openGR()
            if(eef_goal=="place"):
                self.closeGR()


    def pick_and_place2(self):

        for eef_goal in ["home","makis","sakis","makis"]:        
            pose = list_to_pose(self.config_eepose[eef_goal])
            self.go_to(pose)
            if(eef_goal=="sakis"):
                self.closeGR()

        for eef_goal in ["home","makis","sakis","makis","home","sleep"]:        
            pose = list_to_pose(self.config_eepose[eef_goal])
            self.go_to(pose)
            if(eef_goal=="sakis"):
                self.openGR()


        # for eef_goal in ["home_candle","pre_place","place","home_candle","pre_grab","grab","pre_grab","home_candle"]:        
        #     pose = list_to_pose(self.config_eepose[eef_goal])
        #     self.go_to(pose)
        #     if(eef_goal=="grab"):
        #         self.openGR()
        #     if(eef_goal=="place"):
        #         self.closeGR()


    def random_pose(self):
        # repeat random positions
        self.group.set_random_target()
        self.group.go(wait=True)
        self.openGR()

        self.group.set_named_target("Sleep")
        self.group.go(wait=True)
        self.closeGR()


    def go_to(self,pose_goal):

        self.group.set_pose_target(pose_goal)
        self.group.go(wait=True)

        self.group.stop()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.05)

      # repeat yaml joint poses
        # for joint_goal in self.config_joints.keys():
        #     self.group.go(self.config_joints[joint_goal], wait=True)

        # repeat yaml eef poses
        # for eef_goal in self.config_eepose.keys():      
        # for eef_goal in ["home","home_candle","pre_grab","grab","home_candle","pre_place","place"]:        
        #     pose = list_to_pose(self.config_eepose[eef_goal])
        #     self.group.set_pose_target(pose)
        #     self.group.go(wait=True)

        #     if(eef_goal=="grab"):
        #         self.closeGR()
        #     if(eef_goal=="place"):
        #         self.openGR()
        

        
        # pose = list_to_pose([0.19+random.random()*0.05, 0.0, 0.19 +random.random()*0.01, 0.0, 0.0, 0.0, 1.0])
        # self.group.set_pose_target(pose)
        # self.group.go(wait=True)

        # for eef_goal in ["sleep","home"]:        
        #     pose = list_to_pose(self.config_eepose[eef_goal])
        #     (plan, fraction) = self.group.compute_cartesian_path(
        #                                 [pose],   # waypoints to follow
        #                                 0.005,        # eef_step
        #                                 0.0)         # jump_threshold
        #     rospy.loginfo("JIMAS")
        #     rospy.loginfo("fraction --> %d",fraction)
        #     self.display_trajectory(plan)
        #     if(fraction>0.9):
        #         self.group.execute(plan, wait=True)

        # self.openGR()
        # self.closeGR()

    def closeGR(self):
        # self.scene.attach_box()
        self.group_gripper.set_named_target("Home")
        self.group_gripper.go(wait=True)
    def openGR(self):
        # self.scene.detach_box("graspObject")
        self.group_gripper.set_named_target("Open")
        self.group_gripper.go(wait=True)


    def all_close(self,goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    rospy.logerr("MAKIS: PX100 failed to be accurate")
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

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
        # rospy.loginfo("Finding path")
        # self.plan, fraction = self.plan_cartesian_path()
        # rospy.loginfo("path FOUND")
        # self.display_trajectory(self.plan)

        # play all saved moves 
        for joint_state in self.config_joints:
            print(joint_state)

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
        # for joint_goal in self.waypoints_joints:
        #     self.group.go(joint_goal, wait=True)
        #     self.group.stop()

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
        # res = self.object_exists("obstacle")
        # if(res):
        #     self.remove_obj("obstacle")
        # self.add_obstacle()

        # res = self.object_exists("graspObject")
        # if(res):
        #     self.remove_obj("graspObject")
        # self.add_graspObject()

        self.group.set_named_target("Home")
        self.group.go()
        msg = SetBoolResponse()
        msg.message = " "
        msg.success = True
        if(SetBoolRequest.data):
            self.srvf_clearWaypoints(EmptyRequest)
            msg.message = "waypoints reset"
        return msg

    """ SERVICE saveWaypoints
    add new point in parameters server based 
    on joints and eef pose
    """
    def srvf_saveWaypoints(self,EmptyRequest):

        # save joints states
        name_id = "pt"+str(len(self.config_joints)+1)
        self.config_joints[name_id] = self.get_joint_state()
        rospy.set_param("/waypoints_joints/"+name_id,self.config_joints[name_id])

        # save eef pose
        name_id = "pt"+str(len(self.config_eepose)+1)
        self.config_eepose[name_id] = pose_to_list(self.get_eef_pose(False))
        rospy.set_param("/waypoints_eepose/"+name_id,self.config_eepose[name_id])

        return EmptyResponse()

    """SERVICE clearWaypoints
    Reset waypoints list and parameters
    """
    def srvf_clearWaypoints(self,EmptyRequest):
        self.waypoints = []
        rospy.delete_param("/waypoints/")
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