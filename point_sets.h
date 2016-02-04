#ifndef POINT_SETS_H_INCLUDED
#define POINT_SETS_H_INCLUDED

#include <set>
#include <vector>

#include "AprilTags/TagDetector.h"
#include <Eigen/Geometry>

#include "plot_tools.h"


class PointSets
{
private:
    std::vector<Eigen::Vector3d> m_baseSet;
    std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > m_completeSet;
    std::vector<int> m_hullIndices;
    std::pair<double, int> m_minPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_minPointY; // Contains value and index in baseSet vector
    std::pair<double, int> m_maxPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_maxPointY; // Contains value and index in baseSet vector
    cv::Mat m_baseImg;
    std::string m_windowName{"Point Sets Image"};
    PlotTools m_plotTools;

public:
    PointSets(vector<AprilTags::TagDetection> aprilDetections, cv::Mat baseImg):
    m_baseImg{baseImg}
    {
        for (auto point : aprilDetections)
        {
            m_baseSet.push_back(Eigen::Vector3d(point.cxy.first, point.cxy.second, 0));
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
    void generateConvexHullIndices();
    void showSetImage();
};


bool isRightTurn(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);


// Taken from here: http://www.cplusplus.com/reference/set/set/set/
// Sorts the elements in DESCENDING order
struct pairComparison
{
    bool operator() (std::pair<double, int> lhs, std::pair<double, int> rhs) const
        { return lhs.first > rhs.first; }
};


#endif // POINT_SETS_H_INCLUDED
