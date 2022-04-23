// Reference: Gao VSLAM Book 2nd Chinese Ed. Ch 13
//
// Modified by: MT
// First Edit: 2022-April-15
// Previous Edit: 2022-April-23

#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"
#include <iostream>

namespace myslam {

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }

    backend_->Stop();
    std::cin.get(); // wait for a key press
    viewer_->Close();

    auto frames_all = map_->GetAllKeyFrames(); // obtain stored key frames for plotting
    auto points_all = map_->GetAllMapPoints(); // obtain stored key points for exporting to file
    std::cout << "\nTotal num of frames stored: " << frames_all.size() << std::endl;
    std::cout << "Total num of points stored: " << points_all.size() << std::endl;


    ////////// Plotting the estimated and groundtruth trajectories //////////
    // Note: the pose is T_cw, and we want T_wc when plotting
    // T_wc = [R_wc | t_w^{cw}], the translation is from the world frame origin
    // to the current camera frame origin expressed in the world frame.
    // In this setting, we can directly plot the translation from each T_wc to
    // obtain the (estimated) trajectory of the camera (as well as the vehicle).
    TrajectoryType poses_truth, poses_estimated;
    for (unsigned int i = 0; i < frames_all.size(); i++) { // index access works here because the first part of each frames_all element is a frame ID starting from 0
        poses_estimated.push_back(frames_all[i]->pose_.inverse());
    }
    poses_truth = ReadTrajectory(Config::Get<std::string>("groundtruth_dir")); // Read groundtruth from file
    DrawTrajectory(poses_truth, poses_estimated, 1); // >0 - to plot axes of estimated poses; <=0 for not plotting camera axes
    std::cin.get(); // wait for a key press to end the VO process


    ////////// Storing the estimated poses, points, and point cloud //////////
    std::cout << "Saving all keypoints to file." << std::endl;
    WritePointsToFile(Config::Get<std::string>("outputPoints_dir"), points_all);
    std::cout << "All points saved.\n" << std::endl;

    // to save results as point cloud
    // a "save_pointcloud" int flag stored in the config file to determine if the follows are executed
    if (Config::Get<int>("save_pointcloud") > 0) { // flag sets to 1 as true, 0 as false
      std::cout << "Start saving camera centers and point cloud." << std::endl;
      WriteCloudToPLYFile(Config::Get<std::string>("outputCloud_dir"), frames_all, points_all);
      std::cout << "Finished saving point cloud.\n" << std::endl;
    }

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}


}  // namespace myslam
