# homework02-Dimitrios Chamzas 11/2020

# Brief overview
This package(arm_move) is part of Embedded systems in Robotics homework.
The purpose is to control the robot px100 using MoveIt Python API.
The project is heavily depend on MoveIt & interbotix packages.
I used the tutorial interbotix supports for there product as a start point, this is the reason that there is aslo the node `<pkg arm_move/arm_tutorial.py>` that is basicaly there tutorial [link to intebotix git](https://github.com/Interbotix/interbotix_ros_arms/tree/master/interbotix_examples/interbotix_moveit_interface)

# Set Up
The setup of our world is that the robot sits on top of a table that is on top of another table. In front of it there is an object that we consider as an obstacle. Right from this obstacle (in respect to the robot) there is another object. The goal is to figure out a path in order to grab the object and while avoiding our box obstacle leave the the object on the other side.

![](https://github.com/ME495-EmbeddedSystems/homework-3-jimas95/blob/arm/gifs/rViz_arm.gif)


# Nodes

One main node exists `<RobotPX.py>` under the `<pkg arm_move/nodes/mover.py>`.
This node initialize the MoveIt APK and sets up everithing we need in order to control the robot.
## main services 

1. /px100/get_joint_states --> type:Trigger | return current joint state
2. /px100/get_eef_pose     --> type:Trigger | return end effector position
3. /px100/reset'           --> type:SetBool | resets world obstacles and set robot to home position, if bool --> True removes all stored waypoints
4. /px100/saveWaypoints    --> type:Empty   | saves waypoints into parameter server namespace /waypoints/points/pt1-N
5. /px100/clearWaypoints   --> type:Empty,  | clear waypoint list, delete parameter server points.
6. /px100/stop             --> type:Empty   | stop the infinete loop of follow service(if active) 
7. /px100/step             --> type:step_srv| executes (if possible) a path plan from current position to set position args --> Pose : goal position , bool : at end state have gripper open/close.
This is one of the main services we are able to set the position of the eef and if a plan if found to that position the robot will execute the trajectory
8. /px100/follow           --> type:SetBool | starts from current position and executes a path plan for all waypoints and finish at the home position, if Bool--> True itterated infinetly throught the waypoints.
Another main service is the follow which will execute every planing from current position to every waypoint there is in the parameter server under the namespace /waypoints/points/.

In order to clarify what are the waypoints. we use the parameter server and keep some points position there, the step service adds new goal points and the follow service goes to each on of them. 
Waypoints are stored under the `<waypoints/points/pt/>` namespace, you might notice another set of points `</points/pt/>` this are the ones that we load from a yaml file. Every pt has 7 numbers first 3 corespond to x,y,z position and the last 4 are for the orientetion of the eef in quatrenio values x,y,z,w.

# Launch files 
One main launch file exists that calls everything we need.

## run launch file
1. `<roslaunch arm_move arm_box.launch robot_name:=px100 use_fake:=True >`, will use a fake node instead of the real one
2. `<roslaunch arm_move arm_box.launch robot_name:=px100 use_actual:=True >` will use the real robot




## Tests

2 tests where made to check everithing is correct.
1. give the robot a trajectory that will hit the table. Checks that obstacle avoidance works correct (no path should be found and ErrorCode -1 should be returned)
2. give the robot a posible trajectory to execute.

You can run these tests by `<catkin_make run_tests>`. Test files are located under the test folder.


# Robot control 

## from the launch file
After we have executed the launch file we can use the px100 robot.
The launch file auto uploads some points from a yaml file into the parameter server. So the robot is able to execute a trajectory with the follow service. If you want to set your own points you should call the reset service first with an argument of true in order to clear the previous points. Then use the step service to set your new goal points, and last call the follow service that will go to each one of them. If you set follow service arg: True this will cause an infinity loop, from another terminal call stop service in order to stop the follow service.


## some extra commands 
1. torque motors on(default) : `<rosservice call /px100/torque_joints_on`
1. torque motors off : `<rosservice call /px100/torque_joints_off`

# Example 

![](https://github.com/ME495-EmbeddedSystems/homework-3-jimas95/blob/arm/gifs/real_arm.gif)