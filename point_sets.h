#ifndef POINT_SETS_H_INCLUDED
#define POINT_SETS_H_INCLUDED

#include <vector>

#include "AprilTags/TagDetector.h"
#include "gnuplot-iostream.h"

#include "vector2d.h"


class PointSets
{
private:
    std::vector<Point2D> m_baseSet;
    std::vector< std::pair<Point2D, Point2D> > m_completeSet;
    std::pair<double, int> m_minPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_minPointY; // Contains value and index in baseSet vector
    std::pair<double, int> m_maxPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_maxPointY; // Contains value and index in baseSet vector
    cv::Mat m_baseImg;
    std::string m_windowName{"Point Sets Image"};
    Gnuplot m_gp;

public:
    PointSets(vector<AprilTags::TagDetection> aprilDetections, cv::Mat baseImg):
    m_baseImg{baseImg}
    {
        for (auto point : aprilDetections)
        {
            m_baseSet.push_back(Point2D(point.cxy.first, point.cxy.second));
        }
        generateMinMax(); // Generates the minmax values from the loaded baseSet
        cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
    }

    void print();
    void generateMinMax();
    void drawBaseSet();
    void graphBaseSet();
    void generateCompleteSet();
    void graphCompleteSet();
    void drawCompleteSet();
    void showSetImage();
};

#endif // POINT_SETS_H_INCLUDED
