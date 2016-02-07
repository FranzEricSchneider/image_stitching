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
        void generateRandomImage(int maxSize = 1000, int minSize = 400);
        void generateBlankImage(int x, int y);
        void generateRandomDetections(int number);
        void generateDetection(int x, int y);
        void writeRandomDetectionsToFile();
        void pullRandomDetectionsFromFile();
        int randomNumber(int vMax = 1000, int vMin = 0);
        std::random_device m_rd;
        std::mt19937 m_mersenne;
        const std::string fileName{"RandomPointsOutput.csv"};
        std::ofstream m_outFile;
        std::ifstream m_inFile;

    public:
        vector<AprilTags::TagDetection> m_randomDetections;
        cv::Mat1d m_img;

        RandomPoints(std::string placeholder): m_inFile(fileName)
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
