#  Exercise_4
1. Generates a circular path of the work area in front of the robot lying in the Y-Z plane with fixed orientation so that the flange is in the same plane
2. Parameterize this path in time with a method of your choice (reusing existing time parameterization algorithms or coding your own), so that the speed and acceleration limits are respected
3. Visualize your robot running this trajectory in Rviz
4. Publish the resulting trajectory on a topic in such a way that joint positions, velocities and accelerations are plotted in RQT in separate graphs and graphically verifies that the limits (of velocity and acceleration) are respected

# Start the demo

## How to run
1.Build your space:
catkin build smartsix_controller

2.Start the simulation of the Comau Smart-Six robot:
source devel/setup.bash
roslaunch smartsix_moveit_config demo_gazebo.launch

3.Run the planner: 
source devel/setup.bash
roslaunch planning_pkg trajectory_plan.launch

4.Run rqt_multiplot (whit saved configuration):
rosrun rqt_multiplot rqt_multiplot


Referements:
https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
