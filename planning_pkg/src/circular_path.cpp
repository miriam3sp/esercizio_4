#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>


int main(int argc, char **argv)
{
    // Initializing the node and the move_group interface
    ros::init(argc, argv, "smartsix_controller");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Instantiate publisher 
    ros::Publisher trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("trajectory_topic", 10000, true);

    /* Setup
     * MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
     * the `JointModelGroup`.
     */
    static const std::string PLANNING_GROUP = "smartsix";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // The :planning_scene_interface:`PlanningSceneInterface`
    // class is used to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



    /* Visualization
     * ^^^^^^^^^^^^^
     * The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
     * and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
     */
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "smartsix_controller demo", rvt::WHITE, rvt::XXLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();



    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("smartsix_controller demo", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("smartsix_controller demo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("smartsix_controller demo", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));



    
    // Start planning
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");
   
    /* Cartesian Path
     * ^^^^^^^^^^^^^^^
     * We plan a Cartesian path by giving some waypoints,
     * then ask the planner to interpolate between them to have a higher number of waypoints that we can use for planning.
     */

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setStartStateToCurrentState();

    geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    // Trajectory parameters (circle)
    double angle_resolution= 24;
    double d_angle = angle_resolution*3.14/180;
    double angle= 0;
    double radius = 0.2;

    //Plan for the trajectory
    for (int i= 0; i<(360/angle_resolution); i++)
    {
        angle+= d_angle;
        target_pose.position.y = 0.2 + radius*cos(angle);
        target_pose.position.z = 1.170 + radius*sin(angle);
        waypoints.push_back(target_pose);

    }


    // We want cartesian path to be interpolated at a eef_step
    double eef_step = std::min(0.01,radius*angle_resolution);
        
    
    // message to specify trajectories to joint_trajectory_controller
    moveit_msgs::RobotTrajectory trajectory;


    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    const double jump_threshold = 0.0;

    //const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("smartsix_controller demo", "Visualizing path (%.2f%% achieved)", fraction * 100.0);
  


    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "smartsix");

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    //std::cout << trajectory << std::endl;


    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian path ", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
    visual_tools.trigger();

    
    // Publish the trajectory (for rqt_multiplot)
    ros::Duration sleep_time(0.05);
    
    for(int i=0; i < trajectory.joint_trajectory.points.size(); i++)
    {
        trajectory_publisher.publish(trajectory.joint_trajectory.points[i]);

        sleep_time.sleep();
    }


    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to conclude the demo");


    //plan.trajectory_ = trajectory;
    //move_group.execute(plan); 


    // Show status text in RViz
    visual_tools.publishText(text_pose, "Demo terminated", rvt::WHITE, rvt::XXLARGE);
    visual_tools.trigger();

    // END
    spinner.stop();
    ros::shutdown();

    return 0;
}

