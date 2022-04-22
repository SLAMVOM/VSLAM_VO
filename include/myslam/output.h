/*
 This file is the header file for outputting the estimated keyframe poses and stored landmarks

 Reference: VSLAM Book Ch9

 Author: MT
 Creation Date: 2022-April-22
 Previous Edit: 2022-April-22
*/

#pragma once

#include "myslam/config.h"
#include "myslam/common_include.h"
#include "myslam/rotation.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>

using namespace std;

