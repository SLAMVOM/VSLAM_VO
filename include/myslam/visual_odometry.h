// Reference: Gao VSLAM Book 2nd Chinese Ed. Ch 13
//
// Modified by: MT
// First Edit: 2022-April-15
// Previous Edit: 2022-April-22

#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

#include "myslam/groundtruth.h" // added by MT on 2022-April-15
#include "myslam/output.h" // added by MT on 2022-April-22

namespace myslam {

/**
 * VO external interface
 */
class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        /// constructor with config file
        VisualOdometry(std::string &config_path);

        /**
         * perform initialization before running
         * @return true if success
         */
        bool Init();

        /**
         * start vo in dataset
         */
        void Run();

        /**
         * Make a step forward in dataset
         */
        bool Step();

        /// Acquire the frontend status
        FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

    private:
        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        // dataset
        Dataset::Ptr dataset_ = nullptr;
};
} // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_H
