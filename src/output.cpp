/*
 This file is the code file for outputting the estimated keyframe poses and stored landmarks

 Reference: VSLAM Book Ch9

 Author: MT
 Creation Date: 2022-April-22
 Previous Edit: 2022-April-23
*/

#include "myslam/output.h"

// save the extracted keypoints to text file
void WritePointsToFile(const std::string &filename, const LandmarksType &points_all) {
    FILE *fptr = fopen(filename.c_str(), "w"); // writing mode

    if (fptr == NULL) {
        std::cerr << "Error: unable to open file " << filename;
        return;
    }
    assert(!points_all.empty()); // check if there are actually points to be saved
    std::cout << "There are " << points_all.size() << " points to be saved." << std::endl;
    fprintf(fptr, "%ld\n", points_all.size()); // store the total number of points at top of file

    // the key points are stored in a std::unordered_map
    // traverse the unordered_map using an iterator
    for (auto it = points_all.begin(); it!=points_all.end(); it++) {

        unsigned long pt_id = it->second->id_;  // point id stored in the MapPoint object
        Eigen::Vector3d pt = it->second->pos_; // point position is expressed in the world frame

        // store data point into file
        fprintf(fptr, "%ld %g %g %g\n", pt_id, pt[0], pt[1], pt[2]); // each line in the file is a pt in world frame
    }

    fclose(fptr); // close file after finishing storage
}


// save pointcloud as .ply file, a PLY file for inspection in Meshlab or CloudCompare
void WriteCloudToPLYFile(const std::string &filename,
                         const KeyframesType &frames_all,
                         const LandmarksType &points_all) {

    std::ofstream of(filename.c_str());

    of << "ply"
       << '\n' << "format ascii 1.0"
       << '\n' << "element vertex " << frames_all.size() + points_all.size()
       << '\n' << "property float x"
       << '\n' << "property float y"
       << '\n' << "property float z"
       << '\n' << "property uchar red"
       << '\n' << "property uchar green"
       << '\n' << "property uchar blue"
       << '\n' << "end_header" << std::endl;

    // Save the camera centers (as green points) to the output file
    // Note: the poses in frame_all are T_cw, and we want the translation part of T_wc, 
    // so we need to first invert each T_cw then extract the translation vector.
    
    // for (unsigned int i = 0; i < frames_all.size(); i++) {
    //     Eigen::Vector3d t_w_cw = (frames_all[i]->pose_.inverse()).translation();
    //     of << t_w_cw[0] << ' ' << t_w_cw[1] << ' ' << t_w_cw[2]
    //        << " 0 255 0" << '\n';
    // }
    
    // Here does not take the ordering into account
    // If one wants to store the camera center (as well as landmarks) in an ordered sequence,
    // it may be useful to first store the points into a std::map, then output to the result file.
    for (auto it = frames_all.begin(); it!=frames_all.end(); it++) {
        Eigen::Vector3d t_w_cw = (it->second->pose_.inverse()).translation();
        of << t_w_cw[0] << ' ' << t_w_cw[1] << ' ' << t_w_cw[2]
           << " 0 255 0" << '\n';
    }

    // Save the structure (i.e., 3D Points) as white points
    // point position is already expressed in the world frame
    
    // for (unsigned int j = 0; j < points_all.size(); j++) {
    //     Eigen::Vector3d pt = points_all[j]->pos_;
    //     of << pt[0] << ' ' << pt[1] << ' ' << pt[2]
    //        << " 255 255 255\n";
    // }
    for (auto it = points_all.begin(); it!=points_all.end(); it++) {
        Eigen::Vector3d pt = it->second->pos_;
        of << pt[0] << ' ' << pt[1] << ' ' << pt[2]
           << " 255 255 255\n";
    }

    of.close();
}

