# Full Coverage Path Planning 
This project implements a Full Coverage Path Planning (FCPP) algorithm for ROS, allowing a robot to systematically navigate and cover a defined rectangular area. The planner ensures that the entire area is traversed while avoiding obstacles, making it useful for applications such as:

- Autonomous floor cleaning
- Agricultural field coverage
- Surveillance and patrolling
- Autonomous mapping

## Installation

```
cd ~/your_catkin_ws/src
git clone https://github.com/bugrahanturk/full-coverage-path-planning.git
```

## The system consists of two ROS nodes:

### EuclidPlanner (Coverage Path Planner)

Computes a systematic path that ensures full coverage of a user-defined rectangular area.
Uses a sweeping motion (lawnmower pattern) to navigate efficiently.
Avoids obstacles using costmap integration.

### PointPublisher (User-Defined Rectangle Selection)

Allows users to select four points in RViz to define a coverage area.
Publishes these points as a Float32MultiArray message to the planner node.
This setup allows dynamic and flexible path planning for full coverage navigation.


### How It Works
1️⃣ User Defines Coverage Boundaries

The user clicks 4 points in RViz to define a rectangular coverage area.
The PointPublisher node collects these points and publishes them as a ROS message (Float32MultiArray).

2️⃣ Full Coverage Path Computation

The EuclidPlanner node listens to the rectangle_points topic.
The planner:
Identifies the bounding coordinates (xmin, xmax, ymin, ymax).
Creates a sweeping motion path (lawnmower pattern) within this area.
Ensures full coverage while avoiding obstacles using costmap data.
Publishes the path as nav_msgs/Path for execution.
Visualizes the area and planned route in RViz.

3️⃣ Obstacle Handling

The planner checks costmap data for obstacles.
If an obstacle is detected:
The path adjusts dynamically to maintain coverage.
Cells marked as obstacles are avoided.
The robot can navigate around obstacles while continuing coverage.


### Running Nodes
```
rosrun polygon_publisher polygon_publisher
```
<p>
<img src="full_coverage_path/imgs/point_publisher.png" width="874"/>
</p>


```
roslaunch turtlebot3_navigation move_base.launch
```
<p>
<img src="full_coverage_path/imgs/fcpp.png" width="874"/>
</p>

### Results
<p>
<img src="full_coverage_path/imgs/ornek1.png" width="374"/>
</p>

<p>
<img src="full_coverage_path/imgs/ornek2.png" width="874"/>
</p>

<p>
<img src="full_coverage_path/imgs/costmap.png" width="874"/>
</p>

### Visualization of Obstacles
<p>
<img src="full_coverage_path/imgs/obstacles.png" width="374" height="300"/>
</p>

## Conclusion 
This project successfully implements a Full Coverage Path Planning (FCPP) system using ROS and costmap data. The algorithm enables efficient and systematic navigation, making it ideal for real-world autonomous applications.

✔ Customizable coverage areas improve adaptability for different tasks.

✔ Costmap integration ensures obstacle-aware path planning.

✔ Efficient lawnmower pattern reduces redundant coverage.

✔ Visualization in RViz provides real-time feedback.
