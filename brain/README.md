# PACKAGE BRAIN
The brain package is the package that launches everything(PX100, TurtleBuilder), controls, and commands what action each robot should take and when. 

# Overview
This package will either use(launch) both robots, and select fake node or real robots. The package is also heavily depented on the builder and arm packages.

# Nodes
1. Nodes line,pyramid,ramp,smiley_face will build/print different contrustions
2. kill_all creates a safety service that will kill all nodes and actions ensuring the robot will stop moving
3. time managment if launched is a node that automates the printing process to minumum human interfere


## Launch file 
Only one main launch file is needed: 'launch_launAll.launch' . 
Either set use_real or use_fake to true. But one of them must be true. Additionally if use_fake==True you need to set use_loca to false, since there will be no lidar sensor. For more arguments see directly the launch file

# Examples of launch file 
1. 'roslaunch brain launch_launAll.launch use_real:= True'
2. 'roslaunch brain launch_launAll.launch use_gazebo:= True'
3. 'roslaunch brain launch_launAll.launch use_fake:= True use_loca:=False'

