#ifndef DELAUNEY_TRIANGULATION_H_INCLUDED
#define DELAUNEY_TRIANGULATION_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Geometry>
#include <vector>

#include "delaunay_line.h"
#include "delaunay_point.h"
#include "pair_comparison.h"


// TODO: FIND THE RIGHT WAY TO DEFINE PI. Eigen? std?
#define PI 3.141596


// Sorts the vector so the leftmost point is first in the set
bool vector3dComparison(Eigen::Vector3d lhs, Eigen::Vector3d rhs);


class DelaunayTriangulation
{
    private:
        const int m_numPoints;

        void triangulate();
        void completelyConnectSet();
        void mergeGroups(DelaunayTriangulation leftSide, DelaunayTriangulation rightSide);
        void copyConnectionsToThisMap(DelaunayTriangulation subDT);
        DelaunayLine findFirstLine(DelaunayTriangulation &leftSide, DelaunayTriangulation &rightSide);
        DelaunayLine getBaseEdge(const DelaunayPoint &point, DelaunayTriangulation &dt, bool pointIsOnLeft);
        DelaunayPoint getCandidate(const DelaunayLine &line, DelaunayTriangulation &dt, bool isLeftCandidate);
        void populateLeftCandidateSet(const DelaunayLine &line, DelaunayTriangulation &dt,
                                      std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet);
        void populateRightCandidateSet(const DelaunayLine &line, DelaunayTriangulation &dt,
                                       std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet);
        bool circleContainsPoint(const DelaunayPoint &edgePoint, const DelaunayLine &edgeLine, const DelaunayPoint &innerPoint);
        double getCCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison);
        double getCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison);

    public:
        std::map<int, DelaunayPoint> m_pointMap;
//        std::map<int, DelaunayLine> m_lineMap;

        DelaunayTriangulation(std::map<int, DelaunayPoint> pointMap):
        m_numPoints{static_cast<int>(pointMap.size())}, m_pointMap{pointMap}
        {
            triangulate();
        }

        DelaunayTriangulation(std::vector<Eigen::Vector3d> baseSet):
        m_numPoints{static_cast<int>(baseSet.size())}
        {
            std::sort(baseSet.begin(), baseSet.end(), vector3dComparison);
            int counter{};
            for (auto pt: baseSet)
            {
                DelaunayPoint x{counter, pt[0], pt[1]};
                m_pointMap.insert( std::pair<int, DelaunayPoint> (x.m_idx, x) );
                ++counter;
            }
            triangulate();

            /*FOR DEBUGGING*/
//            for (auto pt: baseSet)
//            {
//                std::cout << "baseSet point (x: " << pt[0] << ", y: " << pt[1] << ")\n";
//            }
        }

        int pointWithLowestY();
        int pointWithLowestYAboveGivenIdx(int idx);
        std::vector<DelaunayLine> getLines();
        std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > getLinesForDrawingOrGraphing();
};


void calculateCircle(const DelaunayPoint &point, const DelaunayLine &line, double &xCenter, double &yCenter, double &radius);


#endif // DELAUNEY_TRIANGULATION_H_INCLUDED
