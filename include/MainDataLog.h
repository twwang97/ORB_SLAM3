#pragma once

// #include <sstream>      // std::ostringstream
#include <algorithm> // sort
#include <fstream> // ifstream
#include <iostream> // std::cout
#include <chrono> // chrono
#include <vector> // vector
#include <unistd.h> // usleep
#include "txt_logger.h" // modified on Feb. 11, 2023

#include "Thirdparty/Sophus/sophus/se3.hpp"

namespace ORB_SLAM3 {

class MainDataLog {
 public:

    MainDataLog(const std::string &, const std::string &, const std::string &); 
    ~MainDataLog();
    void write_to_end(); 
    void record_iteratively_start_time(int idx); 
    void sleep_until_triggered(int idx, Sophus::SE3f& Tcw);
    //void LoadImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
    //            std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps);
    bool LoadImages(); 
    TxtLogger *trajectoryFile; 

    // Retrieve paths to images
    std::vector<std::string> vstrImageFilenamesRGB_; // path to images
    std::vector<std::string> vstrImageFilenamesD_; // path to images
    std::vector<double> vTimestamps_; // path to timestamps of each image
    std::string strAssociationFilename_; // associated txt file connecting RGB and depth files
    std::string FinalOutputTrajectoryFilename_; 
    std::string DataDirectoryPath_; 
    int nImages_; 
    bool is_txt_finished; 


    // time
#ifdef COMPILEDWITHC14
    std::chrono::steady_clock::time_point t1;
#else
    std::chrono::monotonic_clock::time_point t1;
#endif

#ifdef COMPILEDWITHC14
    std::chrono::steady_clock::time_point t2;
#else
    std::chrono::monotonic_clock::time_point t2;
#endif
    // double tframe
    double current_time_stamp_; 

    double time_to_track, elapsed_time_between_frames; 

    // vector for tracking time statistics
    std::vector<float> vTimesTrack_;  
    // vTimesTrack_.resize(nImages);

};

}