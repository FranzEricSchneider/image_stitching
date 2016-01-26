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
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}


// changing the tag family
void AprilAnalysis::setTagCodes(string s) {
if (s=="16h5") {
  m_tagCodes = AprilTags::tagCodes16h5;
} else if (s=="25h7") {
  m_tagCodes = AprilTags::tagCodes25h7;
} else if (s=="25h9") {
  m_tagCodes = AprilTags::tagCodes25h9;
} else if (s=="36h9") {
  m_tagCodes = AprilTags::tagCodes36h9;
} else if (s=="36h11") {
  m_tagCodes = AprilTags::tagCodes36h11;
} else {
  std::cout << "Invalid tag family specified" << endl;
  exit(1);
}
}


// parse command line options to change default behavior
void AprilAnalysis::parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adtvC:F:H:S:W:E:G:B:D:")) != -1) {
        // Each option character has to be in the string in getopt();
        // the first colon changes the error character from '?' to ':';
        // a colon after an option means that there is an extra
        // parameter to this option; 'W' is a reserved character
        switch (c) {
        case 'h':
        case '?':
            std::cout << usage;
            exit(0);
            break;
        case 'd':
            m_draw = false;
            break;
        case 'C':
            setTagCodes(optarg);
            break;
        case 'F':
            m_fx = atof(optarg);
            m_fy = m_fx;
            break;
        case 'H':
            m_height = atoi(optarg);
            m_py = m_height/2;
             break;
        case 'S':
            m_tagSize = atof(optarg);
            break;
        case 'W':
            m_width = atoi(optarg);
            m_px = m_width/2;
            break;
        case 'D':
            m_deviceId = atoi(optarg);
            break;
        case ':': // unknown option, from getopt
            std::cout << usage;
            exit(1);
            break;
        }
    }

    if (argc > optind) {
        for (int i=0; i<argc-optind; i++) {
            m_imgNames.push_back(argv[optind+i]);
        }
    }
}


void AprilAnalysis::setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw) {
        cv::namedWindow(windowName, 1);
    }
}


void AprilAnalysis::print_detection(AprilTags::TagDetection& detection) const {
    std::cout << "  Id: " << detection.id
              << " (Hamming: " << detection.hammingDistance << ")";
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                            translation, rotation);
    Eigen::Matrix3d F;
    F <<
        1, 0,  0,
        0,  -1,  0,
        0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    std::cout << "  distance=" << translation.norm()
        << "m, x=" << translation(0)
        << ", y=" << translation(1)
        << ", z=" << translation(2)
        << ", yaw=" << yaw
        << ", pitch=" << pitch
        << ", roll=" << roll
        << std::endl;
}


void AprilAnalysis::processImage(cv::Mat& image, cv::Mat& image_gray) {
    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

    // print out each detection
    std::cout << detections.size() << " tags detected:" << std::endl;
    for (int i = 0; i < detections.size(); i++) {
        print_detection(detections[i]);
    }

    // show the current image including any detections
    if (m_draw) {
        for (int i = 0; i < detections.size(); i++) {
            // also highlight in the image
            detections[i].draw(image);
        }
        imshow(windowName, image); // OpenCV call
    }
}


// Load and process a single image
void AprilAnalysis::loadImages() {
    cv::Mat image;
    cv::Mat image_gray;

    for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
        image = cv::imread(*it); // load image with opencv
//        processImage(image, image_gray);
        while (cv::waitKey(100) == -1) {}
    }
}
