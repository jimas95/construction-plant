#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
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
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveGroupPythonIntefaceTutorial(object):

# """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True,log_level=rospy.DEBUG)

        robot_name = rospy.get_param("~robot_name")
        self.robot_name = robot_name

        dof = rospy.get_param("~dof")
        self.dof = dof

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "interbotix_arm"
        group = moveit_commander.MoveGroupCommander(group_name)
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
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def get_joint_state(self):
        # while(True):
        joint_goal = self.group.get_current_joint_values()
        i=0
        for joint in joint_goal: 
            rospy.logdebug("joint "+str(i)+ ": " +str(joint*180/pi))
            i+=1
        rospy.logdebug("------")
        rospy.sleep(0.2)


    def go_to_joint_state(self):

        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):

        group = self.group
        robot_name = self.robot_name

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        if robot_name == "px100":
            pose_goal.position.x = 0.1
            pose_goal.position.z = 0.15

        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):

        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z += scale * 0.1  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.1  # Third move down (z)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):
        group = self.group
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self.box_name
        scene = self.scene
        return True
        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()
        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):

        robot_name = self.robot_name
        box_name = self.box_name
        scene = self.scene


        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = robot_name + "/ee_gripper_link"
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.025, 0.025, 0.05))


        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):

        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'interbotix_gripper'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
    try:
        print( "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
        input()
        tutorial = MoveGroupPythonIntefaceTutorial()

        print( "============ Press `Enter` to execute a movement using a joint state goal ...")
        input()
        tutorial.go_to_joint_state()
        tutorial.get_joint_state()

        print( "============ Press `Enter` to execute a movement using a pose goal ...")
        input()
        tutorial.go_to_pose_goal()

        print( "============ Press `Enter` to plan and display a Cartesian path ...")
        input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        print( "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
        input()
        tutorial.display_trajectory(cartesian_plan)

        print( "============ Press `Enter` to execute a saved path ...")
        input()
        tutorial.execute_plan(cartesian_plan)

        print( "============ Press `Enter` to add a box to the planning scene ...")
        input()
        tutorial.add_box()

        print( "============ Press `Enter` to attach a Box to the Interbotix robot ...")
        input()
        tutorial.attach_box()

        print( "============ Press `Enter` to plan and execute a path with an attached collision object ...")
        input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)

        print( "============ Press `Enter` to detach the box from the Interbotix robot ...")
        input()
        tutorial.detach_box()

        print( "============ Press `Enter` to remove the box from the planning scene ...")
        input()
        tutorial.remove_box()

        print( "============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
