#!/usr/bin/env python


# ************************************ functions handling scene objects  *************************************
import geometry_msgs.msg
import rospy
import time
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from math import pi

class MySceneMoveIt():

    """
    Init of class scene for px100
    create PlanningSceneInterface
    create my world
    """
    def __init__(self,scene_,robot,eef_link):

        self.robot_name = "px100"
        self.scene = scene_
        self.candle_pos = [ -0.028,
                            -0.191,
                             0.010]
        self.eef_link = eef_link
        self.robot = robot
        self.create_my_scene()

    """
    create our scene setup 
    """

    def create_my_scene(self):  
        self.scene.clear()
        rospy.sleep(2)
        self.add_leg([-0.15,0,0],0.5,"tower_1")
        self.add_leg([-0.15,0,0],0.5,"tower_2")
        self.add_table(width=0.81,height=0.81,position=[0.455,0.095,0],name="table_1")
        self.add_table(width=1.20,height=1.20,position=[1.460,0.290,0],name="table_2")
        self.add_obstacle()    
        self.add_graspObject()

    """
    add a "table" at out scene 
    """    
    def add_table(self,width,height,position,name):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2] - 0.05/2 - 0.01
        self.scene.add_box(name, box_pose, size=(width, height, 0.05))

        rospy.loginfo("ADDING OBJECT --> "+ name)
        if(not self.wait_for_state_update(objName = name, box_is_known=True, timeout=5)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ name)

    """
    add leg(streched box) object
    """
    def add_leg(self,position,length,name):
        leg_size = 0.025
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2] + length/2
        self.scene.add_box(name, box_pose, size=(leg_size, leg_size, length))

        rospy.loginfo("ADDING OBJECT --> "+ name)
        if(not self.wait_for_state_update(objName = name, box_is_known=True, timeout=5)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ name)

    """
    add a box that will be our grasping object at the scene
    """
    def add_graspObject(self, frame_id = "/base_link"):

        quad = quaternion_from_euler(pi/2.0,0,0)
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + frame_id
        box_pose.pose.orientation = Quaternion(x=quad[0],y=quad[1],z=quad[2],w=quad[3])
        box_pose.pose.position.x = self.candle_pos[0]
        box_pose.pose.position.y = self.candle_pos[1]
        box_pose.pose.position.z = self.candle_pos[2]
        self.box_name = "graspObject"
        self.scene.add_cylinder(self.box_name,box_pose,height= 0.01,radius=0.035/2.0)
        if(not self.wait_for_state_update(objName = self.box_name, box_is_known=True, timeout=10)):
            rospy.logerr("ERROR ADDING OBJECT --> "+ self.box_name)

    def set_candle_pos(self,pos):
        
        # update candle pos 
        self.candle_pos = pos 

        #remove if exists candle

        # for name in self.scene.getKnownCollisionObjects():
        #     if(name=="graspObject"):
        #         self.scene.removeCollisionObject(name, False)
        # for name in self.scene.getKnownAttachedObjects():
        #     if(name=="graspObject"):
        #         self.scene.removeAttachedObject(name, False)

        # add new candle
        self.add_graspObject()

    """
    add turtleBuilder object(box) as an obstacle
    """
    def add_obstacle(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot_name + "/base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.2
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.075
        self.box_name = "obstacle"
        self.scene.add_box(self.box_name, box_pose, size=(0.14, 0.05,0.15))

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

    def attach_eef_candle(self):

        # obj exists ? 
        if(self.object_exists("graspObject")):
            # delete candle
            self.remove_obj("graspObject")
        
        # create new candle at eef position
        self.candle_pos=[0,0,0]
        self.add_graspObject(frame_id="/ee_gripper_link")

        # attach it 
        self.attach_box()

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
        rospy.logdebug("MYSCENE ==> object exists!")
        return is_known
