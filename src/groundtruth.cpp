/*
 This file is the code file for plotting the estimated poses
 and comparing the grondtruth to the estimated poses

 Reference: VSLAM Book Ch3 and Ch4

 Author: MT
 Creation Date: 2022-April-15
 Previous Edit: 2022-April-20
*/

#include "myslam/groundtruth.h"

// reading the pose from the given file path
// Here, the transformation is from world to camera, Tcw
TrajectoryType ReadTrajectory(const string &path) {
    ifstream fin(path);
    TrajectoryType trajectory;
    if (!fin) {
        cerr << "trajectory " << path << " not found." << endl;
        return trajectory;
    }

    // Note: In the Kitti Visual Odometry datasets, the ground truth poses are given in T_w_cam0 format,
    // i.e., transformations from the left gray camera frame to the inertial camera (a.k.a. world) frame.
    // IMPORTANT: the term "world frame" here means the first camera frame (or frame 0) when the trajectory is about to start.
    // Again, the "world frame" is not the IMU frame but the initial camera frame when the camera is stationary and about to move!!!
    // And the groundtruth sets the initial camera frame to be identity, indicating a stationary frame.
    // Recall that, in the transformation T_wc = [R_wc | t_{w}^{cw}]: Rotation is: R_wc, and 
    // the translation is: t_{w}^{cw}, that is the translation from the world frame origin to the current camera frame origin expressed in the world frame!!!
    // By following the above, after the left multiplication of T_wc to a point expressed in the camera frame, we can get the coordinates of the point in the world frame.
    while (!fin.eof()) {
        double r11,r12,r13,r21,r22,r23,r31,r32,r33, t1,t2,t3;
        fin >> r11 >> r12 >> r13 >> t1    >> r21 >> r22 >> r23 >> t2    >> r31 >> r32 >> r33 >> t3;
        Eigen::Matrix3d rot_truth;
        rot_truth(0,0) = r11;
        rot_truth(0,1) = r12;
        rot_truth(0,2) = r13;
        rot_truth(1,0) = r21;
        rot_truth(1,1) = r22;
        rot_truth(1,2) = r23;
        rot_truth(2,0) = r31;
        rot_truth(2,1) = r32;
        rot_truth(2,2) = r33;

        // convert the rotation matrix to quaternion, then convert to SE3d
        Eigen::Quaterniond q_rot(rot_truth);
        Sophus::SE3d pose_truth(q_rot, Eigen::Vector3d(t1, t2, t3));

        // store the groundtruth poses
        trajectory.push_back(pose_truth);
    }
    return trajectory;
}


// draw the trajectory using Pangolin
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti, int plotAxes = 0) {
    
    // create Pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);

        // plot the groundtruth trajectory
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f); // blue for groundtruth (0.0R,0.0G,1.0B)
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        // plot the estimated trajectory
        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f); // red for estimated (1.0R,0.0G,0.0B)
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        // plot the axes of the estimated poses if required
        Eigen::Vector3d Ow, Xw, Yw, Zw;
        if (plotAxes > 0) {
            for (size_t i = 0; i < esti.size() - 1; i++) {
                // draw the three axes of each pose, w - world; c - camera/vehicle
                Ow = esti[i].translation(); // Note each element in poses is a T_{wc}
                Xw = esti[i] * (0.5 * Eigen::Vector3d(1, 0, 0)); // the scalar controls the axis length
                Yw = esti[i] * (0.5 * Eigen::Vector3d(0, 1, 0));
                Zw = esti[i] * (0.5 * Eigen::Vector3d(0, 0, 1));
                glBegin(GL_LINES);
                glColor3f(0.63, 0.3, 0.41); // plot the x-axis (pitch of car) as dark red color, #a14e6a
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Xw[0], Xw[1], Xw[2]);
                glColor3f(0.1, 0.3, 0.27); // plot the y-axis (yaw of car) as dark green color
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Yw[0], Yw[1], Yw[2]);
                glColor3f(0.17, 0.33, 0.58); // plot the z-axis (roll of car) as dark blue color
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Zw[0], Zw[1], Zw[2]);
                glEnd();
            }
        }

        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
    }
}

