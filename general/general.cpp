//
// Created by redamancy on 2023/3/14.
//

#include "general.h"
/**
 * @brief 将旋转矩阵转化为欧拉角
 * @param R 旋转矩阵
 * @return 欧拉角
*/
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {z, y, x};
}


/**
 * @brief 将欧拉角转换为轴角
 *
 * @param euler 欧拉角
 * @return Eigen::AngleAxis
 */
Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler)
{
    Eigen::AngleAxisd rotVec;
    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));

    rotVec = roll_angle * yaw_angle * pitch_angle;
    return rotVec;
}

/**
 * @brief 计算旋转矩阵
 * @param theta 欧拉角(顺序Roll-Yaw-Pitch)
 * @return 旋转矩阵
*/
Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;
    Eigen::Matrix3d R_y;
    Eigen::Matrix3d R_z;
    // Calculate rotation about x axis
    R_x <<
        1,       0,              0,
            0,       cos(theta[2]),   -sin(theta[2]),
            0,       sin(theta[2]),   cos(theta[2]);
    // Calculate rotation about y axis
    R_y <<
        cos(theta[1]),    0,      sin(theta[1]),
            0,               1,      0,
            -sin(theta[1]),   0,      cos(theta[1]);
    // Calculate rotation about z axis
    R_z <<
        cos(theta[0]),    -sin(theta[0]),      0,
            sin(theta[0]),    cos(theta[0]),       0,
            0,               0,                  1;
    // Combined rotation matrix
    return R_z * R_y * R_x;
}