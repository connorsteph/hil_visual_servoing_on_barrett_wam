#pragma once

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <random>
#include <Eigen/Dense>
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>
using namespace std;

class Simulator
{
  public:
    string tool_link = "wam/wrist_palm_stump_link";
    ofstream grasp_data;
    string file_directory = "/home/cjs/ros_workspaces/wam_sim_bak/";
    string file_name;
    bool teleop_move = false;
    bool next_object = false;
    bool is_spread;
    bool grip_closed;
    int command_count;
    int object_idx;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    const string PLANNING_GROUP = "arm";
    std::vector<moveit_msgs::CollisionObject> grasping_objects;
    std::vector<moveit_msgs::CollisionObject> current_grasping_objects;
    std::vector<moveit_msgs::CollisionObject> table;
    vector<moveit_msgs::ObjectColor> table_color;
    std::vector<float> controller_axes;
    std::vector<int> controller_buttons;
    const int total_joints = 7;
    std::vector<bool> active_buttons_map = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0};
    std::vector<bool> active_axes_map = {1, 1, 1, 1, 1, 0};
    Eigen::Vector3d object_position;
    Eigen::Vector3d object_position_estimate;
    Eigen::Vector3d spherical_position{0.4, M_PI / 6.0, M_PI}; // r, theta, phi
    Eigen::Matrix<double, 7, 1> current_joint_angles;
    vector<double> goal_joint_angles;
    double yaw_offset = 0.0;
    double control_radius;
    ros::Publisher pub_joint_state;
    Simulator(ros::NodeHandle nh_);
    ~Simulator();

    void loop();
    void setCollisionObjects();
    bool sphere_move(const Eigen::VectorXd &control_vec);
    void teleop_grasp();
    int teleop_grasp_step();
    void teleop_servo();
    int teleop_servo_step();
    Eigen::VectorXd calc_step(Eigen::MatrixXd &jacobian, Eigen::VectorXd &error_vec);

  private:
    ros::Subscriber joy_sub;

    void joy_cb(sensor_msgs::Joy::ConstPtr controllerStatePtr)
    {
        const std::vector<float> temp_controller_axes = controllerStatePtr->axes;
        const std::vector<int> temp_controller_buttons = controllerStatePtr->buttons;
        const float deadzone = 0.2;
        bool sentinel = false;
        for (int i = 0; i < temp_controller_axes.size(); ++i)
        {
            if (active_axes_map[i])
            {
                // std::cout << i << temp_controller_axes[i] << std::endl;
                if (fabs(temp_controller_axes[i]) > deadzone)
                {
                    sentinel = true;
                    break;
                }
            }
        }
        if (!sentinel)
        {
            for (int i = 0; i < temp_controller_buttons.size(); ++i)
            {
                if (active_buttons_map[i])
                {
                    if (abs(temp_controller_buttons[i]) > 0)
                    {
                        sentinel = true;
                        break;
                    }
                }
            }
        }
        if (sentinel)
        {
            if (abs(temp_controller_axes[4]) == 1.0)
            {
                controller_axes = temp_controller_axes;
                next_object = true;
            }
            else
            {
                controller_axes = temp_controller_axes;
                controller_buttons = temp_controller_buttons;
                teleop_move = true;
            }
        }
    }
};
