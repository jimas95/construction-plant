
# Brief overview

## Package ARM

The purpose is to control the robot px100, Meanwhile the TurtleBuilder if free to navigate around it. pathplaning, obstacle avoidance, grasping a specific object and finaly having the ability to 'refuel' the TurtleBuilder are some of the main objectives of this package.


# Set Up
using MoveIt Python API.
The project is heavily dependent on MoveIt & interbotix packages. [link to intebotix git](https://github.com/Interbotix)

# Nodes
1. PICK_AND_PLACE
2. CANDLE
3. SCENE


## main services 



# Launch files 
One main launch file exists that calls everything we need.

## run launch file
1. `<roslaunch arm bring_up.launch use_fake:=True >`, will use a fake node instead of the real one
2. `<roslaunch arm bring_up.launch use_actual:=True >` will use the real robot




## Tests



# Robot control 




## some extra commands 
1. torque motors on(default) : `rosservice call /px100/torque_joints_on`
2. torque motors off :  `rosservice call /px100/torque_joints_off`

# Example 

![](https://github.com/jimas95/construction-plant/blob/working_on/img/px100.gif)

![](https://github.com/jimas95/construction-plant/blob/working_on/img/px100_rViz.gif)