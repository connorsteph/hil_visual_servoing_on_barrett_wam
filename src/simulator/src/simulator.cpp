#include <simulator.h>
#include <simulator_utilities.h>
using namespace std;

void Simulator::setCollisionObjects()
{
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d rpy;
    table.resize(1);
    table[0].id = "table";
    table[0].header.frame_id = "";
    table[0].primitives.resize(1);
    table[0].primitives[0].type = table[0].primitives[0].BOX;
    table[0].primitives[0].dimensions.resize(3);
    table[0].primitives[0].dimensions[0] = 1;
    table[0].primitives[0].dimensions[1] = 1;
    table[0].primitives[0].dimensions[2] = 0.1;
    table[0].primitive_poses.resize(1);
    table[0].primitive_poses[0].position.x = 0.8;
    table[0].primitive_poses[0].position.y = 0;
    table[0].primitive_poses[0].position.z = 1.05;
    table[0].operation = table[0].ADD;
    table_color.resize(1);
    moveit_msgs::ObjectColor temp_color;
    temp_color.color.r = 0;
    temp_color.color.g = 0;
    temp_color.color.b = 255;
    temp_color.color.a = 1.0;
    table_color[0] = temp_color;
    grasping_objects.resize(10);
    grasping_objects[0].header.frame_id = "";
    grasping_objects[0].id = "box1";
    grasping_objects[0].primitives.resize(1);
    grasping_objects[0].primitives[0].type = grasping_objects[0].primitives[0].BOX;
    grasping_objects[0].primitives[0].dimensions.resize(3);
    grasping_objects[0].primitives[0].dimensions[0] = 0.1;
    grasping_objects[0].primitives[0].dimensions[1] = 0.05;
    grasping_objects[0].primitives[0].dimensions[2] = 0.2;
    grasping_objects[0].primitive_poses.resize(1);
    grasping_objects[0].primitive_poses[0].position.x = 0.8;
    grasping_objects[0].primitive_poses[0].position.y = 0.3;
    grasping_objects[0].primitive_poses[0].position.z = 1.2;
    grasping_objects[0].operation = grasping_objects[0].ADD;
    grasping_objects[1].header.frame_id = "";
    grasping_objects[1].id = "box2";
    grasping_objects[1].primitives.resize(1);
    grasping_objects[1].primitives[0].type = grasping_objects[1].primitives[0].BOX;
    grasping_objects[1].primitives[0].dimensions.resize(3);
    grasping_objects[1].primitives[0].dimensions[0] = 0.1;
    grasping_objects[1].primitives[0].dimensions[1] = 0.05;
    grasping_objects[1].primitives[0].dimensions[2] = 0.2;
    grasping_objects[1].primitive_poses.resize(1);
    grasping_objects[1].primitive_poses[0].position.x = 0.6;
    grasping_objects[1].primitive_poses[0].position.y = 0.0;
    grasping_objects[1].primitive_poses[0].position.z = 1.2;
    rpy << 0, 0, M_PI / 3.0;
    // cout << rpy << endl;
    quaternion = toQuaternion(rpy);
    grasping_objects[1].primitive_poses[0].orientation.w = quaternion.w();
    grasping_objects[1].primitive_poses[0].orientation.x = quaternion.x();
    grasping_objects[1].primitive_poses[0].orientation.y = quaternion.y();
    grasping_objects[1].primitive_poses[0].orientation.z = quaternion.z();
    grasping_objects[1].operation = grasping_objects[1].ADD;

    grasping_objects[2].header.frame_id = "";
    grasping_objects[2].id = "box3";
    grasping_objects[2].primitives.resize(1);
    grasping_objects[2].primitives[0].type = grasping_objects[2].primitives[0].BOX;
    grasping_objects[2].primitives[0].dimensions.resize(3);
    grasping_objects[2].primitives[0].dimensions[0] = 0.1;
    grasping_objects[2].primitives[0].dimensions[1] = 0.05;
    grasping_objects[2].primitives[0].dimensions[2] = 0.2;
    grasping_objects[2].primitive_poses.resize(1);
    grasping_objects[2].primitive_poses[0].position.x = 0.6;
    grasping_objects[2].primitive_poses[0].position.y = 0.0;
    grasping_objects[2].primitive_poses[0].position.z = 1.2;
    rpy << 0, 0, 0;
    // cout << rpy << endl;
    quaternion = toQuaternion(rpy);
    grasping_objects[2].primitive_poses[0].orientation.w = quaternion.w();
    grasping_objects[2].primitive_poses[0].orientation.x = quaternion.x();
    grasping_objects[2].primitive_poses[0].orientation.y = quaternion.y();
    grasping_objects[2].primitive_poses[0].orientation.z = quaternion.z();
    grasping_objects[2].operation = grasping_objects[2].ADD;

    grasping_objects.resize(10);
    grasping_objects[3].header.frame_id = "";
    grasping_objects[3].id = "box4";
    grasping_objects[3].primitives.resize(1);
    grasping_objects[3].primitives[0].type = grasping_objects[3].primitives[0].BOX;
    grasping_objects[3].primitives[0].dimensions.resize(3);
    grasping_objects[3].primitives[0].dimensions[0] = 0.1;
    grasping_objects[3].primitives[0].dimensions[1] = 0.05;
    grasping_objects[3].primitives[0].dimensions[2] = 0.2;
    grasping_objects[3].primitive_poses.resize(1);
    grasping_objects[3].primitive_poses[0].position.x = 0.7;
    grasping_objects[3].primitive_poses[0].position.y = -0.4;
    grasping_objects[3].primitive_poses[0].position.z = 1.2;
    rpy << 0, 0, M_PI / 3.0;
    quaternion = toQuaternion(rpy);
    grasping_objects[3].primitive_poses[0].orientation.w = quaternion.w();
    grasping_objects[3].primitive_poses[0].orientation.x = quaternion.x();
    grasping_objects[3].primitive_poses[0].orientation.y = quaternion.y();
    grasping_objects[3].primitive_poses[0].orientation.z = quaternion.z();
    grasping_objects[3].operation = grasping_objects[3].ADD;

    grasping_objects[4].header.frame_id = "";
    grasping_objects[4].id = "box5";
    grasping_objects[4].primitives.resize(1);
    grasping_objects[4].primitives[0].type = grasping_objects[4].primitives[0].BOX;
    grasping_objects[4].primitives[0].dimensions.resize(3);
    grasping_objects[4].primitives[0].dimensions[0] = 0.1;
    grasping_objects[4].primitives[0].dimensions[1] = 0.05;
    grasping_objects[4].primitives[0].dimensions[2] = 0.2;
    grasping_objects[4].primitive_poses.resize(1);
    grasping_objects[4].primitive_poses[0].position.x = 0.7;
    grasping_objects[4].primitive_poses[0].position.y = -0.4;
    grasping_objects[4].primitive_poses[0].position.z = 1.2;
    rpy << 0, 0, 0;
    quaternion = toQuaternion(rpy);
    grasping_objects[4].primitive_poses[0].orientation.w = quaternion.w();
    grasping_objects[4].primitive_poses[0].orientation.x = quaternion.x();
    grasping_objects[4].primitive_poses[0].orientation.y = quaternion.y();
    grasping_objects[4].primitive_poses[0].orientation.z = quaternion.z();
    grasping_objects[4].operation = grasping_objects[4].ADD;

    grasping_objects[5].header.frame_id = "";
    grasping_objects[5].id = "cylinder1";
    grasping_objects[5].primitives.resize(1);
    grasping_objects[5].primitives[0].type = grasping_objects[5].primitives[0].CYLINDER;
    grasping_objects[5].primitives[0].dimensions.resize(2);
    grasping_objects[5].primitives[0].dimensions[0] = 0.25;
    grasping_objects[5].primitives[0].dimensions[1] = 0.04;
    grasping_objects[5].primitive_poses.resize(1);
    grasping_objects[5].primitive_poses[0].position.x = 0.8;
    grasping_objects[5].primitive_poses[0].position.y = 0.3;
    grasping_objects[5].primitive_poses[0].position.z = 1.25;
    grasping_objects[5].operation = grasping_objects[5].ADD;

    grasping_objects[6].header.frame_id = "";
    grasping_objects[6].id = "cylinder2";
    grasping_objects[6].primitives.resize(1);
    grasping_objects[6].primitives[0].type = grasping_objects[6].primitives[0].CYLINDER;
    grasping_objects[6].primitives[0].dimensions.resize(2);
    grasping_objects[6].primitives[0].dimensions[0] = 0.25;
    grasping_objects[6].primitives[0].dimensions[1] = 0.04;
    grasping_objects[6].primitive_poses.resize(1);
    grasping_objects[6].primitive_poses[0].position.x = 0.5;
    grasping_objects[6].primitive_poses[0].position.y = 0.3;
    grasping_objects[6].primitive_poses[0].position.z = 1.25;
    grasping_objects[6].operation = grasping_objects[6].ADD;

    grasping_objects[7].header.frame_id = "";
    grasping_objects[7].id = "cylinder3";
    grasping_objects[7].primitives.resize(1);
    grasping_objects[7].primitives[0].type = grasping_objects[7].primitives[0].CYLINDER;
    grasping_objects[7].primitives[0].dimensions.resize(2);
    grasping_objects[7].primitives[0].dimensions[0] = 0.25;
    grasping_objects[7].primitives[0].dimensions[1] = 0.04;
    grasping_objects[7].primitive_poses.resize(1);
    grasping_objects[7].primitive_poses[0].position.x = 0.7;
    grasping_objects[7].primitive_poses[0].position.y = -0.2;
    grasping_objects[7].primitive_poses[0].position.z = 1.25;
    grasping_objects[7].operation = grasping_objects[7].ADD;

    grasping_objects[8].header.frame_id = "";
    grasping_objects[8].id = "cylinder4";
    grasping_objects[8].primitives.resize(1);
    grasping_objects[8].primitives[0].type = grasping_objects[8].primitives[0].CYLINDER;
    grasping_objects[8].primitives[0].dimensions.resize(2);
    grasping_objects[8].primitives[0].dimensions[0] = 0.25;
    grasping_objects[8].primitives[0].dimensions[1] = 0.04;
    grasping_objects[8].primitive_poses.resize(1);
    grasping_objects[8].primitive_poses[0].position.x = 0.5;
    grasping_objects[8].primitive_poses[0].position.y = -0.4;
    grasping_objects[8].primitive_poses[0].position.z = 1.25;
    grasping_objects[8].operation = grasping_objects[8].ADD;

    grasping_objects[9].header.frame_id = "";
    grasping_objects[9].id = "cylinder5";
    grasping_objects[9].primitives.resize(1);
    grasping_objects[9].primitives[0].type = grasping_objects[9].primitives[0].CYLINDER;
    grasping_objects[9].primitives[0].dimensions.resize(2);
    grasping_objects[9].primitives[0].dimensions[0] = 0.25;
    grasping_objects[9].primitives[0].dimensions[1] = 0.04;
    grasping_objects[9].primitive_poses.resize(1);
    grasping_objects[9].primitive_poses[0].position.x = 0.4;
    grasping_objects[9].primitive_poses[0].position.y = 0.0;
    grasping_objects[9].primitive_poses[0].position.z = 1.25;
    grasping_objects[9].operation = grasping_objects[9].ADD;
    grasping_object_ids.resize(10);
    for (int i = 0; i < 10; ++i)
    {
        grasping_object_ids[i] = grasping_objects[i].id;
    }
}

Simulator::Simulator(ros::NodeHandle nh_)
{   
    controller_buttons.resize(10);
    object_idx = 0;
    joy_sub = nh_.subscribe("/joy", 1, &Simulator::joy_cb, this);
    pub_joint_state = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    setCollisionObjects();
    current_grasping_objects.resize(1);
    current_grasping_objects[0] = grasping_objects[object_idx];
    planning_scene_interface.applyCollisionObjects(current_grasping_objects);
    // planning_scene_interface.applyCollisionObjects(table);
    double x = grasping_objects[0].primitive_poses[0].position.x;
    double y = grasping_objects[0].primitive_poses[0].position.y;
    double z = grasping_objects[0].primitive_poses[0].position.z;
    object_position << x, y, z;
    ros::WallDuration(0.3).sleep();
    cout << "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\nEnter logging file name - no spaces.\n";
    getline(cin, file_name);
    bool confirm = false;
    ofstream my_file;
    string confirm_input;
    ifstream infile(file_directory + file_name);
    while (infile.good())
    {
        confirm = false;
        while (!confirm)
        {
            cout << "File exists - overwrite? y/n\n";
            getline(cin, confirm_input);
            if (confirm_input.compare((string) "y") == 0)
            {
                cout << "Confirmed. Overwriting log.\n";
                confirm = true;
            }
            else if (confirm_input.compare((string) "n") == 0)
            {
                cout << "Enter logging file name - no spaces.\n";
                getline(cin, file_name);
                infile = ifstream(file_directory + file_name);
                break;
            }
        }
        if (confirm)
        {
            cout << "Continuing to teleop task.\n\n********************************n\n";
            break;
        }
    }
    my_file.open(file_directory + file_name);
    my_file.close();
}

Simulator::~Simulator()
{
    joy_sub.shutdown();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    vector<string> object_ids;
    object_ids.resize(1);
    object_ids[0] = current_grasping_objects[0].id;
    planning_scene_interface.removeCollisionObjects(grasping_object_ids);
    object_ids[0] = table[0].id;
    planning_scene_interface.removeCollisionObjects(object_ids);
}

void Simulator::teleop_grasp()
{
    // grasping = true;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    teleop_move = false;
    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    moveit::planning_interface::MoveGroup::Plan my_plan;
    Eigen::Vector3d tool_position = move_group.getCurrentState()->getGlobalLinkTransform(tool_link).translation();
    spherical_position = cartesian_to_spherical(tool_position - object_position_estimate);
    vector<double> joints;
    int c;
    // ros::Duration(0.2).sleep();
    // cout << "VALUE1: " << controller_buttons[8] << teleop_move << endl;
    while (true)
    {
        ros::Rate(30).sleep();
        // cout << "VALUE1: " << controller_buttons[8] << teleop_move << endl;
        if (teleop_move)
        {
            cout << "Command number: " << ++command_count << endl;
            move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joints);
            for (int i = 0; i < joints.size(); ++i)
            {
                current_joint_angles[i] = joints[i];
            }
            c = teleop_grasp_step(); // sets global variable goal_joint_angles in sphere move
            move_group.setJointValueTarget(goal_joint_angles);
            // move_group.plan(my_plan);
            if (c != 0)
            {
                cout << "Moving...\n";
                move_group.move();
                cout << "Done.\n";
            }
            switch (c)
            {
            case 0:
                // cout << "Case 0 - finishing\n";
                teleop_move = false;
                return;
            case 2:
                teleop_move = false;
                break;
            }
        }
    }
}

int Simulator::teleop_grasp_step()
{
    Eigen::VectorXd control_vec(4);
    if (controller_buttons[8])
    {
        cout << "Ending grasping control on move " << command_count << endl;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        cout << "Switching to reaching control . . .\n"
             << endl;
        return 0;
    }
    if (controller_buttons[3])
    {
        if (!is_spread)
        {
            cout << "Opening spread" << endl;
            // bhand->open_spread();
            is_spread = true;
            return 2;
        }
    }
    if (controller_buttons[0])
    {
        if (is_spread)
        {
            cout << "Closing spread" << endl;
            // bhand->close_spread();
            is_spread = false;
            return 2;
        }
    }
    if (controller_buttons[2])
    {
        if (grip_closed)
        {
            cout << "Opening" << endl;
            // bhand->open_grasp();
            grip_closed = false;
            return 2;
        }
    }
    if (controller_buttons[1])
    {
        if (!grip_closed)
        {
            cout << "Grasping" << endl;
            // bhand->close_grasp();
            grip_closed = true;
            return 2;
        }
    }
    object_position_estimate[0] += 0.05 * controller_axes[1];
    object_position_estimate[1] += 0.05 * controller_axes[0];
    control_vec[0] = (controller_buttons[6] - controller_buttons[7]);
    control_vec[1] = controller_axes[3];
    control_vec[2] = -controller_axes[2];
    control_vec[3] = controller_buttons[5] - controller_buttons[4];
    if (controller_buttons[9])
    {
        control_vec << 0, 9.99, 0, 0;
    }
    // cout << "Control vec: \n"
    //      << control_vec << endl;
    if (Simulator::sphere_move(control_vec))
    {
        return 2;
    }
    cout << "here\n";
    return 0;
}

bool Simulator::sphere_move(const Eigen::VectorXd &control_vec)
{
    if (control_vec[1] == 9.99)
    {
        cout << "Resetting to initial position" << endl;
        goal_joint_angles = {0, 0, 0, 1.8, 0, 0, 0};
        spherical_position = {0.4, M_PI / 6.0, M_PI};
        yaw_offset = 0.0;
        return true;
    }
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    Eigen::Quaterniond quaternion;
    geometry_msgs::Pose pose_msg;
    Eigen::VectorXd ortn(4);
    Eigen::Vector3d rpy;
    Eigen::VectorXd full_pose(7);
    double delta_radians = M_PI / 15;
    Eigen::Vector3d rel_cart_pos;
    Eigen::Vector3d cart_pos;
    Eigen::Matrix3d rotator;
    Eigen::Vector3d axis;
    Eigen::VectorXd gains(6);
    vector<double> joints;
    Eigen::Vector3d temp_spherical_position;
    temp_spherical_position = spherical_position;
    spherical_position[0] += 0.05 * control_vec[0];
    rel_cart_pos = spherical_to_cartesian(spherical_position);
    if (abs(control_vec[1]) > 0 || abs(control_vec[2]) > 0)
    {
        axis = Eigen::Vector3d::UnitX() * (control_vec[2]) + Eigen::Vector3d::UnitY() * (control_vec[1]);
        axis.normalize();
        rotator = Eigen::AngleAxisd(delta_radians, axis);
        rel_cart_pos = rotator * rel_cart_pos;
        spherical_position = cartesian_to_spherical(rel_cart_pos);
    }

    // pose_msg = get_pose(object_position_estimate, getToolPosition(current_joint_angles, total_joints));
    pose_msg = get_pose(object_position_estimate, kinematic_state->getGlobalLinkTransform(tool_link).translation());
    // cout << pose_msg << endl;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg, tool_link, 1, 0.05);
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joints);
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            goal_joint_angles[i] = joints[i];
        }
        if (command_count != 1)
            yaw_offset = current_joint_angles[6] - goal_joint_angles[6];
    }
    else
    {
        cout << "IK error, yaw offset reset." << endl;
        yaw_offset = 0.0;
    }
    yaw_offset += 2.0 * delta_radians * control_vec[3];
    // cout << "yaw offset: " << yaw_offset << endl;
    // cout << "Spherical position: \n**************\n"
    //      << spherical_position << "\n***********" << endl;

    rel_cart_pos = spherical_to_cartesian(spherical_position);
    cart_pos = rel_cart_pos + object_position_estimate;
    pose_msg = get_pose(object_position_estimate, rel_cart_pos + object_position_estimate);
    // cout << pose_msg << endl;
    found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg, tool_link, 1, 0.05);
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joints);
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            goal_joint_angles[i] = joints[i];
        }
        goal_joint_angles[6] += yaw_offset;
    }
    else
    {
        ROS_INFO("Did not find IK solution with yaw.");
        spherical_position = temp_spherical_position;
        for (int i = 0; i < goal_joint_angles.size(); ++i)
        {
            goal_joint_angles[i] = current_joint_angles[i];
        }
    }
    return true;
}

void Simulator::teleop_servo()
{
    // cout << "VALUE2: " << controller_buttons[8] << teleop_move << endl;
    // grasping = false;
    moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    planning_scene_interface.removeCollisionObjects(grasping_object_ids);
    current_grasping_objects[0] = grasping_objects[object_idx];
    // current_grasping_objects[0] = grasping_objects[object_idx];
    planning_scene_interface.applyCollisionObjects(current_grasping_objects);
    vector<string> object_ids;
    vector<double> joints;
    int c;
    // command_count = 0;
    // start = ros::WallTime::now();
    Eigen::Vector3d tool_position;
    object_ids.resize(1);
    teleop_move = false;
    const std::string PLANNING_GROUP = "arm";
    goal_joint_angles = {0, 0, 0, 1.8, 0, 0, 0};
    move_group.setJointValueTarget(goal_joint_angles);
    // cout << "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\nMoving to initial position..\n";
    // move_group.move();
    cout << "\n\n***********************\nReady to teleop.\n";

    while (true)
    {
        ros::Rate(30).sleep();
        if (next_object)
        {
            if (command_count > 5)
            {
                cout << "Finished on move " << command_count << ".\n";
                task_time = (ros::WallTime::now() - start).toSec();
                cout << "Task time: " << task_time << endl;
                grasp_data.open(file_directory + file_name, ios::app);
                if (grasp_data.is_open())
                {
                    cout << "Logging to " << file_directory + file_name << endl;
                    grasp_data << object_idx << " " << command_count << " " << task_time << endl;
                    grasp_data.close();
                }
            }
            int idx;
            if (controller_axes[4] < 0)
            {
                object_idx = (object_idx + 1) % grasping_objects.size();
            }
            else
            {
                if (object_idx > 0)
                {
                    --object_idx;
                }
                else
                {
                    object_idx = grasping_objects.size() - 1;
                }
            }
            cout << "Switching to object " << object_idx + 1 << " - resetting" << endl;
            object_ids[0] = current_grasping_objects[0].id;
            planning_scene_interface.removeCollisionObjects(grasping_object_ids);
            current_grasping_objects[0] = grasping_objects[object_idx];
            planning_scene_interface.applyCollisionObjects(current_grasping_objects);
            command_count = 0;
            start = ros::WallTime::now();
            goal_joint_angles = {0, 0, 0, 1.8, 0, 0, 0};
            move_group.setJointValueTarget(goal_joint_angles);
            // move_group.plan(my_plan);
            ros::WallDuration(0.3).sleep();
            move_group.move();
            double x = grasping_objects[object_idx].primitive_poses[0].position.x;
            double y = grasping_objects[object_idx].primitive_poses[0].position.y;
            double z = grasping_objects[object_idx].primitive_poses[0].position.z;
            object_position << x, y, z;
            cout << "Reset complete.\n";
            cout << "Ready to teleop.\n";
            next_object = false;
        }
        // cout << "VALUE2: " << controller_buttons[8] << teleop_move << endl;
        if (teleop_move)
        {
            cout << "Command number: " << ++command_count << endl;
            move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joints);
            for (int i = 0; i < joints.size(); ++i)
            {
                current_joint_angles[i] = joints[i];
            }
            c = teleop_servo_step(); // sets global variable goal_joint_angles in sphere move
            move_group.setJointValueTarget(goal_joint_angles);
            // move_group.plan(my_plan);
            cout << "moving.\n";
            move_group.move();
            cout << "done.\n";
            switch (c)
            {
            case 0:
                // cout << "Case 0 - finishing\n";
                return;
            case 2:
                teleop_move = false;
                break;
            }
        }
    }
}

int Simulator::teleop_servo_step()
{
    if (controller_buttons[8])
    {
        cout << "Switching to grasping controls . . ." << endl;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // planning_scene_interface.removeCollisionObjects(grasping_object_ids);
        controller_buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        return 0;
    }
    default_random_engine generator;
    normal_distribution<double> distribution(0.0, 0.5);
    Eigen::VectorXd rand1(7);
    Eigen::VectorXd rand2(3);
    Eigen::Vector3d control_vec;
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    vector<double> joint_positions;
    Eigen::Vector3d tool_position;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::Vector3d error_vec;
    Eigen::MatrixXd jacobian(7, 6);
    Eigen::MatrixXd lin_jacobian(7, 3);
    Eigen::MatrixXd jacobian_inv(3, 7);
    Eigen::VectorXd dq(7);
    Eigen::Vector3d dx, dx0, dx1, dx2;
    Eigen::MatrixXd motion_vectors(7, 3);
    Eigen::Vector3d vertical(0, 0, 1);
    Eigen::VectorXd delta_joint_angles(7);
    control_vec[0] = (controller_buttons[6] - controller_buttons[7]);
    control_vec[1] = controller_axes[3];
    control_vec[2] = -controller_axes[2];
    if (controller_buttons[9])
    {
        cout << "Resetting to initial position" << endl;
        goal_joint_angles = {0, 0, 0, 1.8, 0, 0, 0};
        command_count = 0;
        start = ros::WallTime::now();
        return 2;
    }
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_angles);
    tool_position = kinematic_state->getGlobalLinkTransform(tool_link).translation();
    kinematic_state->getJacobian(joint_model_group, joint_model_group->getLinkModel(tool_link), reference_point_position, jacobian);
    lin_jacobian = jacobian.block(0, 0, 3, 7);
    lin_jacobian += rand2 * rand1.transpose();
    error_vec = -(tool_position - object_position);
    pseudoInverse(lin_jacobian, jacobian_inv);
    dq = jacobian_inv * error_vec;
    joint_positions.resize(7);
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        joint_positions[i] = current_joint_angles[i] + dq[i];
    }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
    object_position_estimate = kinematic_state->getGlobalLinkTransform(tool_link).translation();
    // cout << object_position - object_position_estimate << endl;
    for (int i = 0; i < joint_positions.size(); ++i)
    {
        joint_positions[i] = current_joint_angles[i] + 0.1 * dq[i];
    }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
    kinematic_state->enforceBounds();
    dx = kinematic_state->getGlobalLinkTransform(tool_link).translation() - tool_position;
    dx2 = dx / dx.norm();
    dx1 = dx.cross(vertical);
    dx0 = dx1.cross(dx2);
    motion_vectors.col(0) = jacobian_inv * dx0;
    motion_vectors.col(1) = jacobian_inv * dx1;
    motion_vectors.col(2) = jacobian_inv * dx2;
    // cout << "Control vec: " << control_vec << endl;
    delta_joint_angles = motion_vectors.col(0) * control_vec[1] + motion_vectors.col(1) * control_vec[2] - 0.1 * dq * control_vec[0];
    for (int i = 0; i < goal_joint_angles.size(); ++i)
    {
        goal_joint_angles[i] = delta_joint_angles[i] + current_joint_angles[i];
    }
    return 2;
}

main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulator");
    cout << "Ros initialized\n";
    ros::NodeHandle nh_("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    Simulator Sim(nh_);
    while (true)
    {
        Sim.teleop_servo();
        Sim.teleop_grasp();
    }
    return EXIT_SUCCESS;
}