/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <opencv2/core/core.hpp>

#include "System.h" // ORB_SLAM3::System SLAM

// #include "txt_logger.h" // modified on Feb. 11, 2023
#include "main_utils.h" // modified on Feb. 11, 2023
#include "MainDataLog.h" // modified on Feb. 11, 2023

void showCurrentImage(cv::Mat& img1);

bool loadRGBDimages(cv::Mat& imgRGB, cv::Mat& imgD, 
                    float imageScale, 
                    std::string DirectoryPath, 
                    std::string& ImageFileNamesRGB_i, std::string& ImageFileNamesD_i);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        std::cerr << std::endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << std::endl;
        return 1;
    }
    
    // mainIO(dataset_folder, associated_txt, trajectory_output_txt)
    ORB_SLAM3::MainDataLog mainIO(argv[3], argv[4], "results/associated_trajectory.txt"); 
    if (!mainIO.LoadImages())
        return 1;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true); // true 
    float imageScale = SLAM.GetImageScale();
    

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni = 0; ni < mainIO.nImages_; ni++)
    {
        // Read image and depthmap from file
        if (!loadRGBDimages(imRGB, imD, imageScale, mainIO.DataDirectoryPath_, 
                        mainIO.vstrImageFilenamesRGB_[ni], mainIO.vstrImageFilenamesD_[ni]))
            return 1;

        mainIO.record_iteratively_start_time(ni); 
        
        // Pass the image to the SLAM system
        Sophus::SE3f Tcw = SLAM.TrackRGBD(imRGB,imD,mainIO.current_time_stamp_);
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f t = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        //printf("\tmainKFtraj,\tt:(%.3f,%.3f,%.3f),\tq:(%.4f,%.4f,%.4f,%.4f)\n", 
        //                       t(0), t(1), t(2), q.x(), q.y(), q.z(), q.w());

        mainIO.sleep_until_triggered(ni, Twc); 
        // break; 

    } // end of main loop
    // std::cerr << "sleep 300 sec" << std::endl; 
    // usleep(15*1e6);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    mainIO.write_to_end(); 
    SLAM.SaveTrajectoryTUM("results/CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("results/KeyFrameTrajectory.txt");   

    return 0;
}

bool loadRGBDimages(cv::Mat& imgRGB, cv::Mat& imgD, float imageScale, 
                    std::string DirectoryPath, std::string& ImageFileNamesRGB_i, std::string& ImageFileNamesD_i){

    // Read image and depthmap from file
    imgRGB = cv::imread(DirectoryPath+"/"+ImageFileNamesRGB_i, cv::IMREAD_UNCHANGED); 
    imgD = cv::imread(DirectoryPath+"/"+ImageFileNamesD_i, cv::IMREAD_UNCHANGED);

    if(imgRGB.empty())
    {
        std::cerr << std::endl << "Failed to load image at: "
                << std::string(DirectoryPath) << "/" << ImageFileNamesRGB_i << std::endl;
        return false;
    }

    if(imageScale != 1.f)
    {
        int width = imgRGB.cols * imageScale;
        int height = imgRGB.rows * imageScale;
        cv::resize(imgRGB, imgRGB, cv::Size(width, height));
        cv::resize(imgD, imgD, cv::Size(width, height));
    }

    showCurrentImage(imgRGB);
    return true; 
} 

void showCurrentImage(cv::Mat& img1){

    cv::namedWindow(CURRENT_FRAME_WINDOW_NAME); /////////
    int down_width = 320;
    int down_height = 240;
    cv::Mat resized_down;
    cv::resize(img1, resized_down, cv::Size(down_width, down_height), cv::INTER_LINEAR);
    cv::imshow(CURRENT_FRAME_WINDOW_NAME,resized_down); //////////
    cv::waitKey(2);

}
