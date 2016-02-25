#ifndef POINT_SETS_H_INCLUDED
#define POINT_SETS_H_INCLUDED


#include <set>
#include <vector>

#include "AprilTags/TagDetector.h"
#include <Eigen/Geometry>

#include "delaunay_triangulation.h"
#include "pair_comparison.h"
#include "plot_tools.h"


class PointSets
{
private:
    std::vector<Eigen::Vector3d> m_baseSet;
    std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > m_completeSet;
    std::vector<int> m_hullIndices;
    std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > m_convexHull;
    std::pair<double, int> m_minPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_minPointY; // Contains value and index in baseSet vector
    std::pair<double, int> m_maxPointX; // Contains value and index in baseSet vector
    std::pair<double, int> m_maxPointY; // Contains value and index in baseSet vector
    cv::Mat m_baseImg;
    std::string m_windowName{"Point Sets Image"};
    PlotTools m_plotTools;
    void drawSet(std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > givenSet);
    void graphSet(std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > givenSet);

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
            m_baseSet.push_back( Eigen::Vector3d{point.cxy.first, point.cxy.second, 0} );
        }
        generateMinMax(); // Generates the minmax values from the loaded baseSet
        cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
//        cv::namedWindow(m_windowName, cv::WINDOW_FULLSCREEN);

        DelaunayTriangulation dt{m_baseSet};
        drawSet(dt.getLinesForDrawingOrGraphing());
        drawDelaunayCircumcircles(dt);
    }

    void print();
    void generateMinMax();
    void drawBaseSet();
    void generateCompleteSet();
    void graphCompleteSet();
    void drawCompleteSet();
    void generateConvexHullIndices();
    void generateConvexHull();
    void drawConvexHull();
    void graphConvexHull();
    void generateDelaunay();
// TODO: MAKE dt A MEMBER VARIABLE SO IT DOESN'T NEED TO BE PASSED
    void drawDelaunayCircumcircles(DelaunayTriangulation &dt);
    void showSetImage();
};


bool isRightTurn(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);


#endif // POINT_SETS_H_INCLUDED
