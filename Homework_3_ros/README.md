#Homework3_ROS

## How to RUN
1. Step-1  
  copy src folder into a empty catkin workspace.
  $ catkin_make

2. Step-2  
- terminal1  
$ roscore
-  terminal2  
$ source devel/setup.bash  
$ rviz
- terminal3  
$ source devel/setup.bash  
$ roslaunch grid_xxxx_xxxx demo_node

3. Step-3 In Rviz  
- Rviz->config->src/gird_xxx_xxx/launch/rviz_config/demo.rviz  
- ADD Goal 3d  
- Use Goal 3D Nav to config a 3D Goal
