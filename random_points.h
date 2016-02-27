#ifndef RANDOM_POINTS_H_INCLUDED
#define RANDOM_POINTS_H_INCLUDED

#include <ctime>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "AprilTags/TagDetector.h"

class RandomPoints
{
    private:
        int m_numberOfPoints;
        const int m_minX{}, m_minY{};  // set to 0
        int m_maxX, m_maxY;  // set randomly in generateRandomImage
        std::random_device m_rd;
        std::mt19937 m_mersenne;
        const std::string fileName{"RandomPointsOutput.csv"};
        std::ofstream m_outFile;
        std::ifstream m_inFile;

        void generateRandomImage(const int maxSize = 2000, const int minSize = 1800);
        void generateBlankImage(const int maxX, const int maxY);
        void generateRandomDetections(const int number);
        void generateDetection(const int x, const int y);
        void writeRandomDetectionsToFile();
        void pullRandomDetectionsFromFile();
        int randomNumber(const int vMax, const int vMin = 0);

    public:
        vector<AprilTags::TagDetection> m_randomDetections;
        cv::Mat m_img;

        RandomPoints(): m_inFile(fileName)
        {
            pullRandomDetectionsFromFile();
            generateBlankImage(m_maxX, m_maxY);
        }

        RandomPoints(int number): m_numberOfPoints{number}, m_mersenne(m_rd()),
                                  m_outFile(fileName)
        {
            generateRandomImage();
            generateRandomDetections(number);
            writeRandomDetectionsToFile();
        }
};

#endif // RANDOM_POINTS_H_INCLUDED
