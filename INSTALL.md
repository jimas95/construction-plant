# Install-project

## Repositories 

* construction-plant : [repo](https://github.com/jimas95/construction-plant/tree/working_on)
* interbotix arm: [repo](https://github.com/Interbotix), [instructions](https://nu-msr.github.io/me495_site/pincherX100.html)
* TurtleBot3: [instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)


## Install 

1. Create the workspace and close the relevant packages
  ```
  download file project.rosinstall
  mkdir -p ~/project_ws/src
  mv ./project.rosinstall ~/project_ws/src
  cd ~/project_ws/src
  wstool init .
  wstool merge -t . ./project.rosinstall
  wstool update
  ```

2. Build the workspace
  ```
  cd ~/project_ws
  catkin init
  catkin build 
  ```


## Repositories:
  ### construction-plant:

    type: git
    url: git@github.com:jimas95/construction-plant.git
    version: main

  
  ### interbotix_ros_core:

    type: git
    url: https://github.com/Interbotix/interbotix_ros_core.git
    version: main
  
  ### interbotix_ros_manipulators:

    type: git
    url: https://github.com/Interbotix/interbotix_ros_manipulators.git
    version: main
  
  ### interbotix_ros_toolboxes:

    type: git
    url: https://github.com/Interbotix/interbotix_ros_toolboxes.git
    version: main

  ### DynamixelSDK:

    type: git
    url: https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    version: master
  ### turtlebot3_msgs:

    type: git
    url: https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    version: master
  ### turtlebot3:

    type: git
    url: https://github.com/ROBOTIS-GIT/turtlebot3.git
    version: master

