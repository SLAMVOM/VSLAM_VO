#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

// Pinhole stereo camera model
class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;

        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
               baseline_ = 0;  // camera intrinsics
        SE3 pose_;              // extrinsics, from stereo camera to inertial world frame
        SE3 pose_inv_;          // inverse of extrinsics

        Camera();

        Camera(double fx, double fy, double cx, double cy, double baseline,
               const SE3 &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse();
        }

        SE3 pose() const { return pose_; }

        // return intrinsic matrix
        Mat33 K() const {
            Mat33 k;
            k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return k;
        }

        // coordinate transform: world, camera, normalized image plane, pixel
        Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w); // from world to camera frame

        Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w); // from camera to world frame

        Vec2 camera2pixel(const Vec3 &p_c); // from camera frame to image pixel coordinates

        Vec3 pixel2camera(const Vec2 &p_p, double depth = 1); // from pixel coordinates to camera frame (when depth = 1, to normalized image plane)

        Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1); // from pixel coordinates to world frame

        Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w); // from world frame to pixel coordinates
};

}   // namespace myslam
#endif  // MYSLAM_CAMERA_H