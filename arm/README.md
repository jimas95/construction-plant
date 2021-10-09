
# Brief overview

## Package ARM

The purpose is to control the robot px100 using MoveIt Python API.
The project is heavily depend on MoveIt & interbotix packages. [link to intebotix git](https://github.com/Interbotix)

# Set Up
The setup of our world is that the robot sits on top of a table that is on top of another table. In front of it there is an object that we consider as an obstacle. Right from this obstacle (in respect to the robot) there is another object. The goal is to figure out a path in order to grab the object and while avoiding our box obstacle leave the the object on the other side.




# Nodes

One main node exists `<RobotPX.py>` under the `<pkg arm/nodes/mover.py>`.
This node initialize the MoveIt APK and sets up everithing we need in order to control the robot.
## main services 



# Launch files 
One main launch file exists that calls everything we need.

## run launch file
1. `<roslaunch arm bring_up.launch robot_name:=px100 use_fake:=True >`, will use a fake node instead of the real one
2. `<roslaunch arm bring_up.launch robot_name:=px100 use_actual:=True >` will use the real robot




## Tests



# Robot control 




## some extra commands 
1. torque motors on(default) : `rosservice call /px100/torque_joints_on`
2. torque motors off :  `rosservice call /px100/torque_joints_off`

# Example 

![](https://github.com/jimas95/construction-plant/blob/working_on/img/px100.gif)

![](https://github.com/jimas95/construction-plant/blob/working_on/img/px100_rViz.gif)