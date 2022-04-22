/*
 This file is the header file for plotting the estimated poses
 and comparing the grondtruth to the estimated poses

 Reference: VSLAM Book Ch3 and Ch4

 Author: MT
 Creation Date: 2022-April-15
 Previous Edit: 2022-April-20
*/

#pragma once

#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <myslam/common_include.h>

using namespace std;

// define a type to store the pose SE3 matrices
typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

// function to draw the trajectory using Pangolin
void DrawTrajectory(const TrajectoryType &gt,
                    const TrajectoryType &esti,
                    int plotAxes); // plotAxes > 0 plots the axes of estimated frame

// function to read the (groundtruth and/or estimated) poses from a given directory
// Here, the transformation matrix is raveled into a 12-D vector, 9-rotation + 3-translation
TrajectoryType ReadTrajectory(const string &path);
