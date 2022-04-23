/*
 This file is the code file for outputting the estimated keyframe poses and stored landmarks

 Reference: VSLAM Book Ch9

 Author: MT
 Creation Date: 2022-April-22
 Previous Edit: 2022-April-22
*/

#include "myslam/output.h"

// save extracted keypoints to text file
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

