# PACKAGE BUILDER

This package controls, navigates and localizes a differential robot which is a different configuration of turtlebot.


# Overview
This package support real robot, gazebo simulation and fake node. It is also heavily dependent on main turtlebot packages.

# Nodes
1. heater --> create an srv that switches a pin on rasp where this eventually opens and closes the circuit for the heating bed
2. odom_noise --> republish fake data to create artificial noise (used only for testing).
3. path_plan --> creates a custom simple action that drives the robot in circles, lines, or waypoints. A variety of markers is also being published.
4. navigate --> create a custom action that receives an x,y goal and navigates the turtlebuilder there.


## Launch file 
Only one main launch file is needed: 'launch_builder.launch' . 
Either set use_real, use_gazebo or use_fake to true. But one of them must be true. Additionally if use_fake==True you need to set use_loca to false, since there will be no lidar sensor. For more arguments see directly the launch file

# Examples of launch file 
1. 'roslaunch builder launch_bring_up.launch use_real:= True'
2. 'roslaunch builder launch_bring_up.launch use_gazebo:= True'
3. 'roslaunch builder launch_bring_up.launch use_fake:= True use_loca:=False'

# Map the environment
If you want to map a new environment you need to edit the localization.launch file and either select slam_toolbox or gmaping.
