#ifndef APRIL_ANALYSIS_H_INCLUDED
#define APRIL_ANALYSIS_H_INCLUDED

/*
* Code altered from this file:
* @file april_tags.cpp
* @brief Example application for April tags library
* @author: Michael Kaess
*/

#include <iostream>
#include <cstring>
#include <vector>
#include <list>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

#include <cmath>
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;


// Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t);
// Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);


class AprilAnalysis {

    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;


    bool m_draw;      // draw image and April tag detections?
    int m_width;      // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx;      // camera focal length in pixels
    double m_fy;
    double m_px;      // camera principal point
    double m_py;
    int m_deviceId;   // camera id (in case of multiple cameras)

    std::list<string> m_imgNames;

public:
    const string usage = "\n"
      "Usage:\n"
      "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
      "\n"
      "Options:\n"
      "  -h  -?          Show help options\n"
      "  -d              Disable graphics\n"
      "  -C <bbxhh>      Tag family (default 36h11)\n"
      "  -D <id>         Video device ID (if multiple cameras present)\n"
      "  -F <fx>         Focal length in pixels\n"
      "  -W <width>      Image width (default 640, availability depends on camera)\n"
      "  -H <height>     Image height (default 480, availability depends on camera)\n"
      "  -S <size>       Tag size (square black frame) in meters\n"
      "\n";
    const char* windowName = "AprilAnalysis";

    // default constructor
    AprilAnalysis() :
        // default settings, most can be modified through command line options (see below)
        m_tagDetector(NULL),
        m_tagCodes(AprilTags::tagCodes36h11),

        m_draw(true),
        m_width(640),
        m_height(480),
        m_tagSize(0.166),
        m_fx(600),
        m_fy(600),
        m_px(m_width/2),
        m_py(m_height/2),

        m_deviceId(0)
        {}

    void setTagCodes(string s);
    void parseOptions(int argc, char* argv[]);
    void setup();
    void print_detection(AprilTags::TagDetection& detection) const;
    void processImage(cv::Mat& image, cv::Mat& image_gray);
    void loadImages();
}; // AprilAnalysis

#endif // APRIL_ANALYSIS_H_INCLUDED
