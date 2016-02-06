#ifndef RANDOM_POINTS_H_INCLUDED
#define RANDOM_POINTS_H_INCLUDED

#include <ctime>
#include <cstdint>
#include <random>
#include <vector>

#include "AprilTags/TagDetector.h"

class RandomPoints
{
    private:
        int m_minX{}, m_minY{};  // set to 0
        int m_maxX, m_maxY;  // set randomly in generateRandomImage
        void generateRandomImage(int maxSize = 1000, int minSize = 400);
        void generateRandomDetections(int number);
        int randomNumber(int vMax = 1000, int vMin = 0);
        std::random_device m_rd;
        std::mt19937 m_mersenne;

    public:
        vector<AprilTags::TagDetection> m_randomDetections;
        cv::Mat1d m_img;

        RandomPoints(int number = 20): m_mersenne(m_rd())
        {
            generateRandomImage();
            generateRandomDetections(number);
        }
};

#endif // RANDOM_POINTS_H_INCLUDED
