title: Motion Planning

# Overview
This document describes the motion planning and control subsystem for Horus. It needs to enable the drone to perform navigation in complex and cluttered environments and also find efficient paths on the macro-scale.

The input into this subsystem is a waypoint anywhere in space, which comes from the Task Planning subsystem upstream (still in progress). The outputs of this subsystem are low level commands that can be executed by the Ardupilot flight controller, which in this case are linear and angular velocity commands.

This module is divided into three layers: 
- `GlobalPlanner` determines the global waypoints the drone needs to follow to avoid macro-scale, static obstacles.
- `LocalPlanner` tries to get the drone to the next global waypoint and avoids unforeseen and dynamic obstacles on the way.
- `TrajectoryController` receives path from the `Local Planner` and follows it.
These are all implemented as ROS2 nodes and are located in `src/nodes/*`

![[Pasted image 20250524133106.png]]
The image above shows the obstacle voxels in red, global goal as the teal sphere, raw local plan as series of blue spheres, and a cleaned up local plan as the thin green line.
# Nodes/Layers
## Local Planner
### Approach and Reasoning
Due to the requirement of navigating through highly cluttered environments, A* was selected. Typically, it is not common to use A* for local planning due to its lack of dynamic considerations and poor performance in reacting to dynamic obstacles, but the priority for the project is in finding optimal paths in complex environments, even at slower speeds.

Dynamic Window Approach (DWA) is a popular local planner that was also considered. It samples viable linear and angular velocity commands and determines which ones is most favorable in terms of avoiding obstacles and reaching the goal quickly. 

However DWA was not selected due to its extremely myopic behavior and suboptimal motions in cluttered environments. High frequency local A* is able to achieve greater optimality in path choice, at the cost of ready dynamic feasibility. In the future, to alleviate this downside, trajectory smoothing will be employed to convert the A* path into a dynamically feasible one, but for now, it works well for slow flight and guarantees deterministic, optimal paths.

Timed Elastic Band (TEB) is another method that can be added in the future. It wasn't selected for the first prototype because as it is based on nonlinear optimization, it is not guaranteed to converge to an optimal path in certain scenarios where an initial guess is poor. Thus, A* was developed first, and the obtained line segments can in the future be used to warm-start TEB with a good initial guess.

Lastly, for certain approaches in the future where convex decomposition will be helpful, A* will still be useful as a seed for this process to extract polyhedrons. An example of this being implemented successfully was in the paper [[SUPER.pdf]] where they also do local planning with A* and then use that as a seed for geenrating a 



### Mechanism Details
The local planner runs A* on a voxel grid map representation. 
	Read more about it here: [[Voxel Grid Map]]

Although in theory it could be run to reach any goal, the A* algorithm is limited to run within a cubic local region centered around the drone. This allows the algorithm to execute a search within the local region in only around 0.0008 seconds with a local region cube edge length of 32 voxels.

The exact start and goal positions are appended to the beginning and end of the path, so that even though A* runs on a voxel grid, it is able to output in exact world coordinates.

After the waypoints are obtained, the path is jagged and not ready for use quite yet:
==INSERT IMAGE OF RAW WAYPOINTS==
The waypoints are post-processed by removing redundant ones. This process is explained below under "Clean Path" in "Additional Algorithms". After this, the resulting path looks something as follows and is ready to be passed to the `Trajectory Controller`: 


### Future Improvements
Speeding up A* with something like Jump Point Search.

Switching to D* or another grid search algorithm that is more suitable for dynamic environments.

Switching to Hybrid A* for kinematics considerations.

As mentioned previously, the best improvements will probably come from integrating A* with some trajectory smoothing or optimization.

## Global Planner

> [!Note]
> This layer is currently under development, but will use PRM on a bounded volumes map.

### Motivation
The Local Planner is able to provide an effective path towards a goal point, but on its own, it is highly susceptible to global suboptimality or even getting stuck in certain cases.

The local planner on its own will get stuck in situations where obstacles are larger than the local planner region (essentially its horizon) and so the local planner can't see a way past it. It may look something like this:
![[Drawing Local Minima|1000]]

In situations where large obstacles are present, even if the drone doesn't get stuck, it may be very efficient in getting around them with only a local planner. The comparison between having and not having a global planner may look as follows.
Globally suboptimal by lacking global planner:
![[Drawing Suboptimal Globally|1000]]

Globally optimal:
![[Drawing Optimal Globally|1000]]

### Mechanism
The Global Planner layer operates on a lower resolution map than the voxel grids. It only needs to encode macro-scale obstacles such as buildings or massive trees.

These can be computed by overlaying object detection from camera feed with range from ORBSLAM3 in the [[Perception]] subsystem. This would allow obtaining a representation of the macro-scale obstacle without needing to be close enough to it to show up in Realsense depth camera data.
- A dedicated page will be written to document this macro-scale mapping once it is implemented.

After a bounded volume map is produced with macro-scale obstacles, a probabilistic roadmap (PRM) can be constructed. This involves sampling points in free space and then connecting them into a graph data structure. Global planning can then run on this PRM, which will not be expected to change since macro-scale obstacles are assumed to be static.
## Trajectory Controller
The final step in achieving drone motion is to send commands for desired velocity. This is done by sending [Twist message](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) commands from the `TrajectoryController` ROS2 node to Ardupilot. 

Ardupilot SITL (software-in-the-loop) comes with the ability to handle this sort of input on the `/ap/cmd_vel` ROS2 topic, which gets bridged to MAVLINK messages. For the purposes of this software stack, letting Ardupilot handle the low level control is a favorable separation of concerns. However, this would change in the future if aggressive maneuvers are desired, which require direct motor-specific thrust control. For those cases, the built in velocity controller within Ardupilot may not prove responsive enough for tracking.

### Mechanism
The mechanism used for this controller is pure pursuit with a lookahead point. This approach was chosen for its reliability and simplicity. With minimal tuning, good results are achievable in a clean manner. 

It works by locating points of intersection between the path segments and a sphere of radius `lookahead_distance` about the drone. The point that is furthest along the path is selected and becomes the `desired_point`. 
![[Drawing Lookahead Intersect|500]]

In the situation that there is no intersection at all, where the drone is far from the path, it will simply direct it towards the nearest projected point on the path to get the drone back on course as fast as possible. It will still keep the desired point constrained to the lookahead sphere, though.
In this way, the error magnitude is always such that it doesn't exceed the `lookahead_distance`.

![[Drawing Lookahead Projection|500]]

Then a simple PD controller computes the desired linear velocity command, where 
```
error = desired_point - current_point
```
and adds it to the Twist message.

Yaw PD control is overlayed on top of this to align the drone heading toward the lookahead point at all times. This is helpful so that the front-facing Realsense camera can detect any obstacles and respond to them. It adds the desired yaw angular velocity to the same Twist message as before and it is published.

# Additional Algorithms

## Clean Path
Post-processing is required by the 





# Primary Learnings So Far

Previously, when Local Planner gave a path only at the resolution of voxels, there was a big challenge of the drone not aligning with the start of the path.
