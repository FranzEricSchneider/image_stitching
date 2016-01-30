/*
 * Code altered from this file:
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 */


#include "april_analysis.h"


inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}


void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
    roll  = standardRad(atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}


void AprilAnalysis::printDetection(AprilTags::TagDetection& detection) const {
    std::cout << "  Id: " << detection.id
              << " (Hamming: " << detection.hammingDistance << ")";
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                            translation, rotation);
    Eigen::Matrix3d F;
    F << 1, 0,  0,
         0,  -1,  0,
         0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    std::cout << "  distance = " << translation.norm()
        << "m,\nx = " << translation(0)
        << "\ny = " << translation(1)
        << "\nz = " << translation(2)
        << "\nyaw = " << yaw
        << ", pitch = " << pitch
        << ", roll = " << roll
        << std::endl;
}


void AprilAnalysis::processImage()
{
    cv::Mat image_gray;
    cv::cvtColor(m_img, image_gray, CV_BGR2GRAY);
    m_detections = m_tagDetector->extractTags(image_gray);
}


void AprilAnalysis::processAndShowImage()
{
    processImage();
    std::cout << m_detections.size() << " tags detected:" << std::endl;
    for (int i = 0; i < static_cast<int>(m_detections.size()); i++) {
        printDetection(m_detections[i]);
    }

    for (int i = 0; i < static_cast<int>(m_detections.size()); i++) {
        m_detections[i].draw(m_img);
    }
    cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
    cv::imshow(m_windowName, m_img);
    while (cv::waitKey(100) == -1) {}
}
