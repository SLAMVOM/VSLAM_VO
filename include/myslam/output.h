/*
 This file is the header file for outputting the estimated keyframe poses and stored landmarks

 Reference: VSLAM Book Ch9

 Author: MT
 Creation Date: 2022-April-22
 Previous Edit: 2022-April-22
*/

#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>

#include "myslam/config.h"
#include "myslam/common_include.h"
#include "myslam/rotation.h"
#include "myslam/map.h"
// #include "myslam/mappoint.h"

using namespace std;
using namespace myslam;

typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType; // Unordered map is an associative container that contains key-value pairs with unique keys
typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType; // Unordered map is an associative container that contains key-value pairs with unique keys

// save the extracted keypoints to text file
void WritePointsToFile(const std::string &filename, const LandmarksType &points_all);

// save pointcloud as .ply file
void WriteCloudToPLYFile(const std::string &filename, const KeyframesType &frames_all, const LandmarksType &points_all);

// // normalize the pointcloud to make the centroid of the cloud to be zero
// void PointCLoudNormalize();