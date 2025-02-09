# euclid_planner

nav_core::BaseGlobalPlanner pluginexample

# Install

```
cd ~/robotlar_ws/src
git clone https://gitlab.com/blm6191_2425b/blm6191/euclid_planner.git
cd ~/robotlar_ws
catkin_make
source ~/.bashrc
```

# Run

Update move_base.launch file at
~~~
~/robotlar_ws/src/turtlebot3/turtlebot3_navigation/launch/move_base.launch
~~~
with EuclidPlanner loaded as the global_planner plugin 
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
...
  	<param name="base_global_planner" value="global_planner/EuclidPlanner"/>  
...
  </node>
```

# Run with AMCL
~~~
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 
~~~

# Run with SLAM
~~~
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation move_base.launch
~~~

# Published Topic

```
nav_msgs::Path /move_base/EuclidPlanner/euclid_plan
```
