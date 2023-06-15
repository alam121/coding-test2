# coding-test2

This includes code for the following question:

a. Use the link “TurtleBot3 (robotis.com)” and clone the Turtlebot (ROS Melodic) simulation.
Follow the steps (6.1, 6.2 & 6.3) to create a 2D map and setup navigation accordingly. Verify that
your navigation is working by giving goals using RVIZ goal tool.

b. Write a C++ node which subscribes to topic “/goal_location” of type “std_msgs/Int8. Further, the
code should be able to publish 3 different goal locations depending on the number published on
the mentioned ROS topic. For example, if “1” is published on the ROS topic, then the robot should
go to location 1 and so on for other locations. If any number is published other than 1,2 or 3, it
should not navigate the robot. You can choose your own goal locations inside the map, just make
sure that the goal locations are inside the map.

# For a)
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Create 2D map using gmapping (please ensure that the packages are installed)
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
Move the robot around using teleop so the map is accurate
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Save the map 

```
rosrun map_server map_saver -f ~/map
```

For Navigation after lanching empty world:
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

1. Click the 2D Pose Estimate button in the RViz menu.
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.
4. Launch keyboard teleoperation node to precisely locate the robot on the map.
5. Click the 2D Nav Goal button in the RViz menu.
6. Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.

![alt text](https://github.com/alam121/coding-test2/blob/main/q2_5.JPG =250x250)

