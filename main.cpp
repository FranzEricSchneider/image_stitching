/*///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//*/

#include <iostream>
#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

#include "april_analysis.h"

bool try_use_gpu = false;
std::vector<cv::Mat> imgs;
std::string result_name = "result.jpg";

void printUsage();
int parseCmdArgs(int argc, char** argv);

int main(int argc, char* argv[])
{
    int retval = parseCmdArgs(argc, argv);
    if (retval) return -1;

    cv::Mat pano;
    cv::Stitcher stitcher = cv::Stitcher::createDefault(try_use_gpu);

    cv::Stitcher::Status status = stitcher.stitch(imgs, pano);

    if (status != cv::Stitcher::OK)
    {
        std::cout << "Can't stitch images, error code = " << status << std::endl;
        return -1;
    }
    std::cout << "Finished processing image!" << std::endl;

//    cv::Mat pano = cv::imread("imgs/a7.jpg"); // a2, a5
    AprilAnalysis april;
    april.setParameters(pano.rows, pano.cols);
    april.setup();
    april.processAndShowImage(pano);

    cv::imwrite(result_name, pano);
    std::cout << "Finished writing image!\n" << std::endl;
    return 0;
}


void printUsage()
{
    std::cout <<
        "Rotation model images stitcher.\n\n"
        "stitching img1 img2 [...imgN]\n\n"
        "Flags:\n"
        "  --try_use_gpu (yes|no)\n"
        "      Try to use GPU. The default value is 'no'. All default values\n"
        "      are for CPU mode.\n"
        "  --output <result_img>\n"
        "      The default is 'result.jpg'.\n";
}


int parseCmdArgs(int argc, char** argv)
{
    if (argc == 1)
    {
        printUsage();
        return -1;
    }
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "/?")
        {
            printUsage();
            return -1;
        }
        else if (std::string(argv[i]) == "--try_use_gpu")
        {
            if (std::string(argv[i + 1]) == "no")
                try_use_gpu = false;
            else if (std::string(argv[i + 1]) == "yes")
                try_use_gpu = true;
            else
            {
                std::cout << "Bad --try_use_gpu flag value\n";
                return -1;
            }
            i++;
        }
        else if (std::string(argv[i]) == "--output")
        {
            result_name = argv[i + 1];
            i++;
        }
        else
        {
            cv::Mat img = cv::imread(argv[i]);
            cv::Mat resizedImg;
            if (img.empty())
            {
                std::cout << "Can't read image '" << argv[i] << "'\n";
                return -1;
            }
            std::cout << "Got image '" << argv[i] << "'\n";
            imgs.push_back(img);
////             cv::Size Dsize{408, 230};
//            cv::Size Dsize{816, 459};
//            cv::resize(img, resizedImg, Dsize);
//            imgs.push_back(resizedImg);
////            cv::imwrite("resized" + std::to_string(i) + ".jpg", resizedImg);
        }
    }
    return 0;
}