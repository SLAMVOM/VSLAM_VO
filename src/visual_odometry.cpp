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
    std::cin.get(); // added by MT, April 15, 2022
    viewer_->Close();
    
    auto frames_all = map_->GetAllKeyFrames(); // added by MT, April 15, 2022
    auto points_all = map_->GetAllMapPoints(); // added by MT, April 15, 2022
    std::cout << "\nTotal num of frames stored: " << frames_all.size() << std::endl; // added by MT, April 15, 2022
    std::cout << "Total num of points stored: " << points_all.size() << std::endl; // added by MT, April 15, 2022

    // Note: the pose is Twc, and we want Tcw when plotting
    TrajectoryType poses_truth, poses_estimated;
    for (int i = 0; i < frames_all.size(); i++) {
        poses_estimated.push_back(frames_all[i]->pose_.inverse());
    } // added by MT, April 15, 2022
    poses_truth = ReadTrajectory(Config::Get<std::string>("groundtruth_dir")); // added by MT, April 15, 2022
    DrawTrajectory(poses_truth, poses_estimated, 1); // added by MT, April 15, 2022; 1 - to plot axes of estimated poses
    std::cin.get(); // added by MT, April 15, 2022

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
