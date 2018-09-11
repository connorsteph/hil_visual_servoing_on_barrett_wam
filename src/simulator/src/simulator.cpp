#include <simulator.h>
#include <simulator_utilities.h>
using namespace std;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // vector<moveit_msgs::ObjectColor> colors;
    // colors.resize(2);
    // colors[0].color.r = 0;
    // colors[0].color.g = 50;
    // colors[0].color.b = 0;
    // colors[0].color.a = 0.9;
    // colors[1].color.r = 50;
    // colors[1].color.g = 0;
    // colors[1].color.b = 0;
    // colors[1].color.a = 0.9;

    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 1;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.8;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    vector<moveit_msgs::CollisionObject> objects;
    objects.resize(1);
    objects[0].header.frame_id = "";
    objects[0].id = "object";

    /* Define the primitive and its dimensions. */
    objects[0].primitives.resize(1);
    objects[0].primitives[0].type = objects[0].primitives[0].BOX;
    objects[0].primitives[0].dimensions.resize(3);
    objects[0].primitives[0].dimensions[0] = 0.1;
    objects[0].primitives[0].dimensions[1] = 0.05;
    objects[0].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    objects[0].primitive_poses.resize(1);
    objects[0].primitive_poses[0].position.x = 0.8;
    objects[0].primitive_poses[0].position.y = 0.3;
    objects[0].primitive_poses[0].position.z = 1.2;

    objects[0].operation = objects[0].ADD;
    
    vector<moveit_msgs::ObjectColor> colors;
    colors.resize(1);
    moveit_msgs::ObjectColor red;
    red.color.r = 0;
    red.color.g = 0;
    red.color.b = 255;
    red.color.a = 1.0;
    colors[0] = red;
    planning_scene_interface.applyCollisionObjects(collision_objects, colors);
    planning_scene_interface.applyCollisionObjects(objects);
}

Simulator::Simulator(ros::NodeHandle nh_)
{
    cout << "Constructor flag\n";

    joy_sub = nh_.subscribe("/joy", 1, &Simulator::joy_cb, this);
    pub_joint_state = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObjects(planning_scene_interface);
    ros::WallDuration(1.0).sleep();
    // while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    // {
    //     sleep_t.sleep();
    // }
}

Simulator::~Simulator()
{
    joy_sub.shutdown();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    vector<string> object_ids = {"object", "table"};
    planning_scene_interface.removeCollisionObjects(object_ids);
}

void Simulator::teleop_grasp()
{
    teleop_move = false;
    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    vector<double> joints;

    cout << "\n**************************************" << endl;
    int i = 1;
    int c;
    goal_joint_angles = {0, 0, 0, 1.8, 0, 0, 0};
    move_group.setJointValueTarget(goal_joint_angles);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        cout << "moving to initial position\n";
        move_group.move();
        cout << "ready.\n";
    }
    else
    {
        object_position[0] -= 0.05 * controller_axes[1];
        object_position[1] -= 0.05 * controller_axes[0];
        spherical_position[0] -= 0.05 * (controller_buttons[6] - controller_buttons[7]);
    }

    while (true)
    {
        ros::Rate(10).sleep();
        if (teleop_move)
        {
            // cout << "received teleop grasp command" << endl;
            cout << "Move: " << i++ << endl;
            move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joints);
            // cout << "Translation: \n"
            //      << move_group.getCurrentState()->getGlobalLinkTransform("wam/wrist_palm_stump_link").translation() << "\n*****************" << endl;
            // cout << "Rotation: \n"
            //      << move_group.getCurrentState()->getGlobalLinkTransform("wam/wrist_palm_stump_link").rotation() << "\n*****************" << endl;
            for (int i = 0; i < joints.size(); ++i)
            {
                current_joint_angles[i] = joints[i];
            }
            c = teleop_grasp_step(); // sets global variable goal_joint_angles in sphere move
            move_group.setJointValueTarget(goal_joint_angles);
            move_group.plan(my_plan);
            cout << "moving.\n";
            move_group.move();
            cout << "done.\n";
            switch (c)
            {
            case 0: // convergence - return early (OR IK FAILURE!!)
                cout << "Case 0 - finishing\n";
                return;
            case 2: // step completed successfully
                // cout << "NEXT!\n";
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
        cout << "User quit teleop. Exiting.." << endl;
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
    object_position[0] += 0.05 * controller_axes[1];
    object_position[1] += 0.05 * controller_axes[0];
    control_vec[0] = (controller_buttons[6] - controller_buttons[7]);
    control_vec[1] = controller_axes[3];
    control_vec[2] = -controller_axes[2];
    control_vec[3] = controller_buttons[5] - controller_buttons[4];
    cout << "Control vec: \n"
         << control_vec << endl;
    if (Simulator::sphere_move(control_vec))
    {
        return 2;
    }
    return 0;
}

bool Simulator::sphere_move(const Eigen::VectorXd &control_vec)
{
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    Eigen::Quaterniond quaternion;
    geometry_msgs::Pose pose_msg;
    Eigen::VectorXd ortn(4);
    Eigen::Vector3d rpy;
    Eigen::VectorXd full_pose(7);
    double delta_radians = M_PI / 15; // 45 degrees per second at 30Hz
    Eigen::Vector3d rel_cart_pos;
    Eigen::Vector3d cart_pos;
    Eigen::Matrix3d rotator;
    Eigen::Vector3d axis;
    Eigen::VectorXd gains(6);
    vector<double> joints;
    spherical_position[0] += 0.05 * control_vec[0];
    rel_cart_pos = spherical_to_cartesian(spherical_position);
    // cout << "Cartesian pos:" << rel_cart_pos << endl;
    if (abs(control_vec[1]) > 0 || abs(control_vec[2]) > 0)
    {
        axis = Eigen::Vector3d::UnitX() * (control_vec[2]) + Eigen::Vector3d::UnitY() * (control_vec[1]);
        axis.normalize();
        rotator = Eigen::AngleAxisd(delta_radians, axis);
        rel_cart_pos = rotator * rel_cart_pos;
        spherical_position = cartesian_to_spherical(rel_cart_pos);
    }
    pose_msg = get_pose(object_position, getToolPosition(current_joint_angles, total_joints));
    // cout << "Pose msg: " << pose_msg << endl;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg, "wam/wrist_palm_stump_link", 1, 0.05);
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joints);
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            goal_joint_angles[i] = joints[i];
        }
        // yaw_offset = current_joint_angles[6] - goal_joint_angles[6];
        yaw_offset -= 3.0 * delta_radians * control_vec[3];
    }
    else
    {
        cout << "IK problem" << endl;
        yaw_offset = 0.0;
    }

    cout << "yaw offset: " << yaw_offset << endl;
    cout << "Spherical position: \n**************\n"
         << spherical_position << "\n***********" << endl;
    rel_cart_pos = spherical_to_cartesian(spherical_position);
    cart_pos = rel_cart_pos + object_position;
    quaternion = inwards_normal_to_quaternion(spherical_position);
    ortn[0] = quaternion.x();
    ortn[1] = quaternion.y();
    ortn[2] = quaternion.z();
    ortn[3] = quaternion.w();
    // full_pose[0] = cart_pos[0];
    // full_pose[1] = cart_pos[1]
    // full_pose[2] = cart_pos[2];
    // full_pose[3] = ortn[0];
    // full_pose[4] = ortn[1];
    // full_pose[5] = ortn[2];
    // full_pose[6] = ortn[3];
    pose_msg.position.x = cart_pos[0];
    pose_msg.position.y = cart_pos[1];
    pose_msg.position.z = cart_pos[2];
    pose_msg.orientation.x = ortn[0];
    pose_msg.orientation.y = ortn[1];
    pose_msg.orientation.z = ortn[2];
    pose_msg.orientation.w = ortn[3];

    found_ik = kinematic_state->setFromIK(joint_model_group, pose_msg, "wam/wrist_palm_stump_link", 1, 0.05);
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
        ROS_INFO("Did not find IK solution");
        for (int i = 0; i < goal_joint_angles.size(); ++i)
        {
            goal_joint_angles[i] = current_joint_angles[i];
        }
    }
    return true;
}

main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulator");
    cout << "Ros initialized\n";
    ros::NodeHandle nh_("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    Simulator Sim(nh_);
    Sim.teleop_grasp();
    return EXIT_SUCCESS;
}