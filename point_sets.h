#ifndef POINT_SETS_H_INCLUDED
#define POINT_SETS_H_INCLUDED


#include <set>
#include <vector>

#include "AprilTags/TagDetector.h"
#include <Eigen/Geometry>

#include "d_triangulation.h"
#include "pair_comparison.h"
#include "plot_tools.h"


class PointSets
{
private:
    std::vector<Eigen::Vector3i> m_baseSet;
    std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > m_completeSet;
    std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > m_convexHull;
    DTriangulation m_dt;  // Delaunay triangulation of the points
    std::pair<double, int> m_minPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_minPointY; // Contains value and index in baseSet vector
    cv::Mat m_baseImg;
    std::string m_windowName{"Point Sets Image"};
    PlotTools m_plotTools;

    void findMinimumsInBaseset();
    void generateConvexHullIndices(std::vector<int> &hullIndices);
    void drawSet(std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > givenSet);
    void graphSet(std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > givenSet);

public:
// TODO: MAKE AN EQUALS OPERATOR FOR dt SO THAT IT DOESN'T HAVE TO RUN EVERY TIME
    PointSets(vector<AprilTags::TagDetection> aprilDetections, cv::Mat baseImg):
    m_baseImg{baseImg}
    {
        if (aprilDetections.size() == 0)
        {
            std::cout << "Failed to make PointSets instance, aprilDetections vector was of size 0\n";
            std::cout << "Terminating execution\n";
            exit(-1);
        }

        for (auto point : aprilDetections)
        {
            m_baseSet.push_back( Eigen::Vector3i{static_cast<int>(point.cxy.first),
                                                 static_cast<int>(point.cxy.second), 0} );
        }
        findMinimumsInBaseset(); // Generates the minmax values from the loaded baseSet
        cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
//        cv::namedWindow(m_windowName, cv::WINDOW_FULLSCREEN);
    }

    void drawBaseSet();
    void generateCompleteSet();
    void drawCompleteSet();
    void graphCompleteSet();
    void generateConvexHull();
    void drawConvexHull();
    void graphConvexHull();
    void generateDelaunay();
    void drawDelaunay();
    void graphDelaunay();
    void drawDelaunayCircumcircles();
    void showImage();
};


bool isRightTurn(Eigen::Vector3i a, Eigen::Vector3i b, Eigen::Vector3i c);


#endif // POINT_SETS_H_INCLUDED
