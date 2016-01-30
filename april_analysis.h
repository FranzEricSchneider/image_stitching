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

/*
 *     <bbxhh>      Tag family (default 36h11)\n"
 *     <id>         Video device ID (if multiple cameras present)\n"
 *     <fx>         Focal length in pixels\n"
 *     <width>      Image width in pixels (default 640)\n"
 *     <height>     Image height in pixels (default 480)\n"
 *     <size>       Tag size (square black frame) in meters\n"
 */

// Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t);
// Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);


class AprilAnalysis
{

    AprilTags::TagDetector* m_tagDetector;
    const AprilTags::TagCodes m_tagCodes{AprilTags::tagCodes36h11}; // Default

    const double m_tagSize{0.166}; // April tag side length in meters of square black frame
    const int m_deviceId{0};       // camera id (in case of multiple cameras)
    const double m_fx{600};        // camera focal length in pixels
    const double m_fy{600};
    const char* m_windowName{"AprilAnalysis"};
    int m_width;             // image size in pixels
    int m_height;
    double m_px;             // camera principal point
    double m_py;

    std::list<std::string> m_imgNames;
    void printDetection(AprilTags::TagDetection& detection) const;

public:
    AprilAnalysis()
        {
            std::string name{"result.jpg"};
//            std::cout << "What image would you like to load? ('string'.jpg): ";
//            std::getline(std::cin, name);
//            name += ".jpg";
            m_img = cv::imread(name);
            if (m_img.empty())
            {
                std::cout << "Couldn't read image '" << name << "'\n";
                exit(1);
            }
            m_height = m_img.rows;
            m_py = m_height/2;
            m_width = m_img.cols;
            m_px = m_width/2;

            m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
        }

    vector<AprilTags::TagDetection> m_detections;
    cv::Mat m_img;                 // Load the image of interest
    void processImage();
    void processAndShowImage();
}; // AprilAnalysis

#endif // APRIL_ANALYSIS_H_INCLUDED
