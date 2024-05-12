#pragma once

#include <sstream>      // std::ostringstream
#include <iomanip> // std::setprecision
#include <iostream> // std::cout

namespace ORB_SLAM3 {

class TxtLogger {
 public:

    TxtLogger(const std::string &filePath);
    void logTitles(); 
    void addNew(double, const std::string&, const std::string&, const std::string&, 
                float tx, float ty, float tz, float qx, float qy, float qz, float qw); 
    void write();
    
    // txt file to be logged
    std::ostringstream log_file; // timings_log

    // Log the timings to the given file.
    std::string log_filePath;
    bool is_log_needed; 
    bool is_txt_open; 

};

}