#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    for (int i = 0; i < joint_names.size(); ++i)
    {
        cout << joint_names[i] << endl;

    }
    // move_group.setEndEffector("wam/wrist_palm_stump_link");
    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Let's set a joint space goal and move towards it.  This will replace the
    // pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //   // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    joint_group_positions[0] = 1.0; // radians
    joint_group_positions[1] = -1.0; // radians
    joint_group_positions[2] = 1.0; // radians
    joint_group_positions[3] = -1.0; // radians
    joint_group_positions[4] = 1.0; // radians
    joint_group_positions[5] = -1.0; // radians

    // joint_group_positions[1] = 0.0; // radians
    // joint_group_positions[0] = 0.0;// radians
    // joint_group_positions[2] = 0.0;// radians
    // joint_group_positions[3] = 0.0; // radians
    // joint_group_positions[4] = 0.0;// radians
    // joint_group_positions[5] = 0.0; // radians
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(my_plan);
    
    move_group.move();
    
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    
    // END_TUTORIAL

    ros::shutdown();
    return 0;
}