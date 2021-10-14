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
from arm.srv import step_srv,step_srvResponse,step_srvRequest
import tf2_ros
from arm.Myscene import MySceneMoveIt
from arm.srv import get_eef_goal,get_eef_goalResponse,get_eef_goalRequest
import time

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
        rospy.init_node('RobotPX',anonymous=True,log_level=rospy.DEBUG)
        # rospy.init_node('RobotPX',anonymous=True)

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
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()
        rospy.logdebug( "PICK AND PLACE ==========> Reference frame: %s" % planning_frame)
        rospy.logdebug( "PICK AND PLACE ==========> End effector: %s" % eef_link)
        rospy.logdebug( "PICK AND PLACE ==========> Robot Groups:" + str(robot.get_group_names()))


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

        # joint state values of gripper open/close 
        self.gripper_state_open = [0.03,-0.03]   
        self.gripper_state_close = [0.036,-0.036] 

        rospy.wait_for_service('get_eef_goal_pick')
        rospy.wait_for_service('get_eef_goal_place')
        self.get_eef_goal_pick  = rospy.ServiceProxy('get_eef_goal_pick' , get_eef_goal)
        self.get_eef_goal_place = rospy.ServiceProxy('get_eef_goal_place', get_eef_goal)

  
    """
    main update loop
    does not do much only debuging mesagges
    everything is being executed and controled using services
    """
    def update(self):
        rate = rospy.Rate(1) # publish freacuancy 
        while not rospy.is_shutdown():
            rospy.logdebug("PICK AND PLACE--> looping!")

            # self.scene.play_scene()
            self.pick_and_place()

            rate.sleep()


    def pick_and_place(self):

        self.pick(self.get_eef_goal_pick())
        self.place(self.get_eef_goal_place())

        self.pick(self.get_eef_goal_place())
        self.place(self.get_eef_goal_pick())


    def pick(self,poses):

        rospy.logdebug("PICK AND PLACE ==> GO TO PREGRASP")        
        self.go_to(poses.pregrasp)

        rospy.logdebug("PICK AND PLACE ==> GO TO GRASP")
        self.go_to(poses.grasp)
        
        rospy.logdebug("PICK AND PLACE ==> CLOSE GRIPPER")
        self.close_gripper()
        self.scene.attach_eef_candle()

        rospy.logdebug("PICK AND PLACE ==> GO TO PREGRASP")
        self.go_to(poses.pregrasp)


    def place(self,poses):

        rospy.logdebug("PICK AND PLACE ==> GO TO PRE PLACE")
        self.go_to(poses.pregrasp)

        rospy.logdebug("PICK AND PLACE ==> GO TO PLACE")
        self.go_to(poses.grasp)

        rospy.logdebug("PICK AND PLACE ==> OPEN GRIPPER")
        self.open_gripper()
        self.scene.detach_box("graspObject")
        rospy.logdebug("PICK AND PLACE ==> GO TO PRE PLACE")
        self.go_to(poses.pregrasp)


    def go_to(self,pose_goal):
        self.group.set_pose_target(pose_goal)
        # add planing first 
        # check if is planning is succesfull
        # then execute
        self.group.go(wait=True)
        time.sleep(0.1) # do I need this ?
        self.group.stop()
        self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.05)

    def get_eef_goal_pick(self):
        try:
            grasping_poses = self.get_eef_goal_pick()
            return grasping_poses
        except rospy.ServiceException as exc:
            rospy.logerr("PICK AND PLACE ==> Service did not process request: " + str(exc))

    def get_eef_goal_place(self):
        try:
            grasping_poses = self.get_eef_goal_place()
            return grasping_poses
        except rospy.ServiceException as exc:
            rospy.logerr("PICK AND PLACE ==> Service did not process request: " + str(exc))

    def close_gripper(self):
        gripper_state = self.group_gripper.get_current_joint_values()
        self.group_gripper.go(self.gripper_state_open,wait=True)

    def open_gripper(self):
        gripper_state = self.group_gripper.get_current_joint_values()
        self.group_gripper.go(self.gripper_state_close,wait=True)



    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    def all_close(self,goal, actual, tolerance):
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    rospy.logerr("PICK AND PLACE: PX100 failed to be accurate")
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True


"""
creates object of class and calls main loop 
"""
if __name__ == '__main__':
    try:
        robot = RobotPX()
        robot.update()
    except rospy.ROSInterruptException:
        pass