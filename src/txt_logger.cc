#include "txt_logger.h"

namespace ORB_SLAM3 {

TxtLogger::TxtLogger(const std::string &filePath){
    log_filePath = filePath; 
    is_txt_open = false; 
    is_log_needed = true; 
    if (log_filePath.empty()) {
        is_log_needed = false; 
    }

    if(!is_txt_open){
        is_txt_open = true; 
        logTitles(); 
    }
    std::cout << "init log_filePath: " << log_filePath << std::endl; 
}

void TxtLogger::logTitles(){
    if (!log_filePath.empty()) {
        log_file << "# associated file path and the corresponding camera poses\n" 
                << "# time_stamp, " 
                // << "dataset_folder, " 
                << "rgb_image, " 
                << "depth_image, "
                << "tx, " 
                << "ty, " 
                << "tz, " 
                << "qx, " 
                << "qy, "
                << "qz, "
                << "qw "
                << std::endl;
    }
}

void TxtLogger::addNew(double timeStamp, const std::string &folder, 
                    const std::string &imgRGB_path, const std::string &imgD_path, 
                    float tx, float ty, float tz, 
                    float qx, float qy, float qz, float qw){
    
    if (!log_filePath.empty()) {
        log_file << std::setprecision(6) << timeStamp << " "
                // << folder << " "
                << imgRGB_path << " "
                << imgD_path << " "
                << tx << " " << ty << " " << tz << " "
                << qx << " " << qy << " " << qz << " " << qw
                << std::endl;
    }
    
}

void TxtLogger::write(){
    if(is_log_needed){
        FILE* file = fopen(log_filePath.c_str(), "wb");
        std::string str = log_file.str();
        fwrite(str.c_str(), 1, str.size(), file);
        fclose(file);
    }
}

}