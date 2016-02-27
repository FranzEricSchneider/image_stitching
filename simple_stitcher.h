#ifndef SIMPLE_STITCHER_H_INCLUDED
#define SIMPLE_STITCHER_H_INCLUDED

#include <iostream>
#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

class SimpleStitcher
{
    cv::Stitcher stitcher;
    std::vector<cv::Mat> imgs;
    cv::Mat panorama;
    std::string prefix{"imgs/"};
    int startNum;
    int stopNum;
    std::string suffix{".jpg"};
    std::string panoramaName;
    double imageScalar;

    void loadImages();

public:
    SimpleStitcher()
    {
        stitcher = cv::Stitcher::createDefault();
        loadImages();
    }

    void stitchImagesAndOutput();
};

bool almostEqual(double n1, double n2);

#endif // SIMPLE_STITCHER_H_INCLUDED
