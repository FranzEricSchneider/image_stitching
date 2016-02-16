#include "simple_stitcher.h"


int SimpleStitcher::stitchImagesAndOutput()
{
    cv::Stitcher::Status status = stitcher.stitch(imgs, pano);
    if (status != cv::Stitcher::OK)
    {
        std::cout << "Can't stitch images, error code = " << status << std::endl;
        return -1;
    }
    cv::imwrite(panoName, pano);
    std::cout << "Finished writing image!\n" << std::endl;
    return 0;
}


int SimpleStitcher::loadImages()
{
    std::cout << "What is the prefix for the images? (imgs/'string'1.jpg): ";
    std::string addPrefix;
    std::getline(std::cin, addPrefix);
    prefix += addPrefix;
    std::cout << "What image number should the pano START with? (imgs/a'int'.jpg): ";
    std::cin >> startNum;
    std::cout << "What image number should the pano END with? (imgs/a'int'.jpg): ";
    std::cin >> stopNum;
    std::cout << "What should the images be scaled with? (double): ";
    std::cin >> imageScalar;
    std::cin.ignore(32767, '\n');
    std::cout << "What should the output pano be called? ('string'.jpg): ";
    std::getline(std::cin, panoName);
    panoName += ".jpg";

    for (int i = startNum; i <= stopNum; ++i)
    {
        std::string name = prefix + std::to_string(i) + suffix;
        cv::Mat img = cv::imread(name);
        if (img.empty())
        {
            std::cout << "Can't read image '" << name << "'\n";
            return -1;
        }
        std::cout << "Got image '" << name << "'\n";
        if (almostEqual(imageScalar, 1.0))
        {
            imgs.push_back(img);
        }
        else
        {
            cv::Mat resizedImg;
            int resizeX = img.cols * imageScalar;
            int resizeY = img.rows * imageScalar;
            cv::Size Dsize{resizeX, resizeY};
            cv::resize(img, resizedImg, Dsize);
            imgs.push_back(resizedImg);
        }
    }
    return 0;
}


bool almostEqual(double n1, double n2)
{
    return fabs(n1 - n2) < 0.0001;
}
