#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
* configuration class, using SetParameterFile to locate configuration file
* then through Get to obtain the corresponding values
* singleton pattern
**/
class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {} // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        // access the parameter values
        template <typename T>
        static T Get(const std::string &key) {
            return T(Config::config_->file_[key]);
        } 
};
}   // namespace myslam

#endif  // MYSLAM_CONFIG_H