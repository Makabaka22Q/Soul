//
// Created by redamancy on 2023/3/14.
//
#pragma once

#include <unistd.h>

#include <iostream>
#include <fstream>

#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <fmt/format.h>
#include <fmt/color.h>

#include <glog/logging.h>

#include "../debug.h"

using namespace std;
using namespace cv;

//相机SN号
const string camera_name = "KE0200020184";
// const string camera_name = "KE0200110077";
// const string camera_name = "KE0210070575";
// const string camera_name = ""JK0210040012"";

enum TargetType {SMALL, BIG, BUFF};  //装甲板类型

/**
 * @brief 存储任务所需数据的结构体
 *
 */
struct TaskData
{
    int mode;    //模式
    double bullet_speed;
    Mat img;
    Eigen::Quaterniond quat;   //四元数
    int timestamp;//单位：ms
};

/**
 * 初始化矩阵，vector->matrix
 * */
template<typename T>
bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
{
    int cnt = 0;
    for(int row = 0;row < matrix.rows();row++)
    {
        for(int col = 0;col < matrix.cols();col++)
        {
            matrix(row,col) = vector[cnt];
            cnt++;
        }
    }
    return true;
}

/**
 * 将旋转矩阵转化为欧拉角
 * */
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

/**
 * 将欧拉角转化为轴角
 * */
Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);

/**
 * 计算旋转矩阵
 * */
Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);