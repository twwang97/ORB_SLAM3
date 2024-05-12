#include "MainDataLog.h"

namespace ORB_SLAM3 {

MainDataLog::MainDataLog(const std::string &DataDirectoryPath, 
                        const std::string &strAssociationFilename,
                        const std::string &FinalOutputTrajectoryFilename){
    
    // Retrieve paths to images
    DataDirectoryPath_ = DataDirectoryPath; 
    FinalOutputTrajectoryFilename_ = FinalOutputTrajectoryFilename; 
    strAssociationFilename_ = strAssociationFilename;

    // Create a txt file for logging
    trajectoryFile = new TxtLogger(FinalOutputTrajectoryFilename_); 
    is_txt_finished = false; 
}

MainDataLog::~MainDataLog(){
    if(!is_txt_finished){
        // Save camera trajectory
        trajectoryFile->write();
        is_txt_finished = true; 
    }
}

void MainDataLog::write_to_end(){

    // Tracking time statistics
    sort(vTimesTrack_.begin(),vTimesTrack_.end());
    float totaltime = 0;
    for(int ni=0; ni < nImages_; ni++)
        totaltime += vTimesTrack_[ni];

    std::cout << "-------" << std::endl << std::endl;
    std::cout << "median tracking time: " << vTimesTrack_[nImages_/2] << std::endl;
    std::cout << "mean tracking time: " << totaltime/nImages_ << std::endl;

    // Save camera trajectory
    trajectoryFile->write();
    is_txt_finished = true; 
}

// void MainDataLog::LoadImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
//               std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps)
bool MainDataLog::LoadImages()
{
    // load RGB-D images and return number of images

    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename_.c_str());
    while(!fAssociation.eof())
    {
        std::string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps_.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB_.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD_.push_back(sD);

        }
    }

    // Check consistency in the number of images and depthmaps
    nImages_ = vstrImageFilenamesRGB_.size();
    if(vstrImageFilenamesRGB_.empty())
    {
        std::cerr << std::endl << "No images found in provided path." << std::endl;
        nImages_ = 0;
    }
    else if(vstrImageFilenamesD_.size()!=vstrImageFilenamesRGB_.size())
    {
        std::cerr << std::endl << "Different number of images for rgb and depth." << std::endl;
        nImages_ = 0;
    }

    // vector for tracking time statistics
    vTimesTrack_.resize(nImages_);

    if (nImages_ < 1)
        return false;
    else
        return true; 

} // MainDataLog::LoadImages

void MainDataLog::record_iteratively_start_time(int idx){
    // tframe = vTimestamps[ni];
    current_time_stamp_ = vTimestamps_[idx];

#ifdef COMPILEDWITHC14
    t1 = std::chrono::steady_clock::now();
#else
    t1 = std::chrono::monotonic_clock::now();
#endif

} // MainDataLog::record_iteratively_start_time

void MainDataLog::sleep_until_triggered(int idx, Sophus::SE3f& Tcw){
#ifdef COMPILEDWITHC14
    t2 = std::chrono::steady_clock::now();
#else
    t2 = std::chrono::monotonic_clock::now();
#endif
    time_to_track =  std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    vTimesTrack_[idx] = time_to_track;
    
    Eigen::Vector3f t = Tcw.translation();
    Eigen::Quaternionf q = Tcw.unit_quaternion();
    // printf("\tmainKFtraj,\tt:(%.3f,%.3f,%.3f),\tq:(%.4f,%.4f,%.4f,%.4f)\n", 
    //                    t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w());
    trajectoryFile->addNew(vTimestamps_[idx], DataDirectoryPath_, vstrImageFilenamesRGB_[idx], vstrImageFilenamesD_[idx], 
                            t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w());
    
    // Wait to load the next frame
    elapsed_time_between_frames = 0;
    if(idx < nImages_-1)
        elapsed_time_between_frames = vTimestamps_[idx+1] - current_time_stamp_;
    else if(idx > 0)
        elapsed_time_between_frames = current_time_stamp_ - vTimestamps_[idx - 1];

    if(time_to_track < elapsed_time_between_frames)
        usleep((elapsed_time_between_frames - time_to_track)*1e6);

} // MainDataLog::record_iteratively_end_time

};