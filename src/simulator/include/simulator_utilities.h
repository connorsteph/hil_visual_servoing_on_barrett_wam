#pragma once
#include <simulator.h>

Eigen::Vector3d spherical_to_cartesian(const Eigen::Vector3d &spherical_cooords)
{
    Eigen::Vector3d cartesian_coords;
    double r = spherical_cooords[0];
    double theta = spherical_cooords[1];
    double phi = spherical_cooords[2];
    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);
    double c_phi = std::cos(phi);
    double s_phi = std::sin(phi);
    cartesian_coords[0] = r * s_theta * c_phi;
    cartesian_coords[1] = r * s_theta * s_phi;
    cartesian_coords[2] = r * c_theta;
    return cartesian_coords;
}

Eigen::Vector3d cartesian_to_spherical(const Eigen::Vector3d &cartesian_coords)
{
    Eigen::Vector3d spherical_coords;
    double x = cartesian_coords[0];
    double y = cartesian_coords[1];
    double z = cartesian_coords[2];
    spherical_coords[0] = std::sqrt(x * x + y * y + z * z);
    spherical_coords[1] = std::atan2(std::sqrt(x * x + y * y), z);
    spherical_coords[2] = std::atan2(y, x);
    return spherical_coords;
}

Eigen::Quaterniond toQuaternion(const Eigen::Vector3d &RPY)
{
    Eigen::Quaterniond q;
    double roll = RPY[0];
    double pitch = RPY[1];
    double yaw = RPY[2];
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    q.w() = cy * cr * cp + sy * sr * sp;
    q.x() = cy * sr * cp - sy * cr * sp;
    q.y() = cy * cr * sp + sy * sr * cp;
    q.z() = sy * cr * cp - cy * sr * sp;
    return q;
}

Eigen::Quaterniond inwards_normal_to_quaternion(const Eigen::Vector3d &spherical_coords)
{ // returns the quaternion that gives the orientation of the inward normal vector of a sphere, at a given phi, theta in spherical coords.
    Eigen::Quaterniond quaternion;
    Eigen::Quaterniond quaternion_2;
    Eigen::Vector3d rpy;
    Eigen::Vector3d u, v, x, y;
    Eigen::Matrix3d SO_3;
    double theta = spherical_coords[1];
    double phi = spherical_coords[2];
    double c_phi = std::cos(phi);
    double c_theta = std::cos(theta);
    double s_phi = std::sin(phi);
    double s_theta = std::sin(theta);

    Eigen::Vector3d vertical(0.0, 0.0, 1.0);
    Eigen::Vector3d inwards_normal((-c_phi * s_theta), (-s_phi * s_theta), (-c_theta));
    x << 1, 0, 0;
    y << 0, 1, 0;
    u = inwards_normal.cross(-y);
    if (u.norm() < 1e-10)
    {
        u << -1, 0, 0;
    }
    v = u.cross(-inwards_normal);
    for (int i = 0; i < 3; ++i)
    {
        SO_3(i, 0) = u[i];
        SO_3(i, 1) = v[i];
        SO_3(i, 2) = inwards_normal[i];
    }
    quaternion = Eigen::Quaterniond(SO_3);
    return quaternion;
}

geometry_msgs::Pose get_pose(const Eigen::Vector3d &object_position, const Eigen::Vector3d &tool_position)
{
    geometry_msgs::Pose pose_msg;
    Eigen::Vector3d rel_position = tool_position - object_position;
    Eigen::Vector3d local_spherical_position = cartesian_to_spherical(rel_position);
    Eigen::Quaterniond quaternion = inwards_normal_to_quaternion(local_spherical_position);
    pose_msg.position.x = tool_position[0];
    pose_msg.position.y = tool_position[1];
    pose_msg.position.z = tool_position[2];
    pose_msg.orientation.x = quaternion.x();
    pose_msg.orientation.y = quaternion.y();
    pose_msg.orientation.z = quaternion.z();
    pose_msg.orientation.w = quaternion.w();
    return pose_msg;
}

Eigen::Vector3d getToolPosition(const Eigen::VectorXd &joint_positions, int dof)
{
    double d3 = 4.76e-2;
    double L3 = 55.8e-2;
    double L4 = 29.1e-2;
    double L7 = 6.5e-2;
    Eigen::VectorXd denavit_a(7);
    denavit_a << 0.0, 0.0, 0.0, d3, -d3, 0.0, 0.0;
    Eigen::VectorXd denavit_alpha(7);
    denavit_alpha << 0, -M_PI / 2.0, M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0;
    Eigen::VectorXd denavit_d(7);
    denavit_d << 0, 0, L3, 0, L4, 0, L7;
    Eigen::Vector3d tool_pos;
    Eigen::Matrix4d temp_matrix;
    Eigen::Matrix4d hom_mat = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix3d rot_mat;
    Eigen::VectorXd cos_theta = joint_positions.array().cos();
    Eigen::VectorXd sin_theta = joint_positions.array().sin();
    Eigen::VectorXd cos_alpha = denavit_alpha.array().cos();
    Eigen::VectorXd sin_alpha = denavit_alpha.array().sin();
    for (int i = 0; i < dof; ++i)
    {
        temp_matrix(0, 0) = cos_theta[i];
        temp_matrix(0, 1) = -sin_theta[i];
        temp_matrix(0, 2) = 0.0;
        temp_matrix(0, 3) = denavit_a[i];
        //
        temp_matrix(1, 0) = sin_theta[i] * cos_alpha[i];
        temp_matrix(1, 1) = cos_theta[i] * cos_alpha[i];
        temp_matrix(1, 2) = -sin_alpha[i];
        temp_matrix(1, 3) = -sin_alpha[i] * denavit_d[i];
        //
        temp_matrix(2, 0) = sin_theta[i] * sin_alpha[i];
        temp_matrix(2, 1) = cos_theta[i] * sin_alpha[i];
        temp_matrix(2, 2) = cos_alpha[i];
        temp_matrix(2, 3) = cos_alpha[i] * denavit_d[i];
        //
        temp_matrix(3, 0) = 0.0;
        temp_matrix(3, 1) = 0.0;
        temp_matrix(3, 2) = 0.0;
        temp_matrix(3, 3) = 1.0;
        hom_mat = hom_mat * temp_matrix;
    }
    for (int j = 0; j < 3; ++j)
    {
        tool_pos[j] = hom_mat(j, 3);
    }
    tool_pos[2] += 1.0;
    return tool_pos;
}

template <typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
    double max = 40.0;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singular_values = svd.singularValues();
    double cond = (singular_values(0) / singular_values(singular_values.size() - 1));
    if (cond > max)
    {
        singular_values(singular_values.size() - 1) = 0;
        double cond = (singular_values(0) / singular_values(singular_values.size() - 2));
        // std::cout << "condition number failed, setting smallest singular value to 0" << std::endl;
    }
    typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * singular_values.array().abs().maxCoeff();
    result = svd.matrixV() * _Matrix_Type_((singular_values.array().abs() > tolerance).select(singular_values.array().inverse(), 0)).asDiagonal() *
             svd.matrixU().adjoint();
    return true;
}