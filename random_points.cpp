#include "random_points.h"


void RandomPoints::generateRandomImage(int maxSize, int minSize)
{
    m_maxX = randomNumber(maxSize, minSize);
    m_maxY = randomNumber(maxSize, minSize);
    // From here: http://stackoverflow.com/questions/31337397/how-to-create-empty-mat-in-opencvs
    m_img = cv::Mat1d(m_maxY, m_maxX, 0.0);
}


void RandomPoints::generateRandomDetections(int number)
{
    for (int i{}; i < number; ++i)
    {
        int x = randomNumber(m_maxX);
        int y = randomNumber(m_maxY);
        AprilTags::TagDetection detection{};
        detection.cxy.first = x;
        detection.cxy.second = y;
        m_randomDetections.push_back(detection);
    }
}


int RandomPoints::randomNumber(int vMax, int vMin)
{
    uint32_t maxVal{-1};
    static const double fraction = 1.0 / static_cast<double>(maxVal);
    return static_cast<int>(m_mersenne() * fraction * (vMax - vMin) + vMin);
}
