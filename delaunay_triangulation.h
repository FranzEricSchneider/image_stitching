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
        DelaunayPoint getLeftCandidate(const DelaunayLine &line, DelaunayTriangulation &dt);
        DelaunayPoint getRightCandidate(const DelaunayLine &line, DelaunayTriangulation &dt);
        bool circleContainsPoint(const DelaunayPoint &edgePoint, const DelaunayLine &edgeLine, const DelaunayPoint &innerPoint);

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


#endif // DELAUNEY_TRIANGULATION_H_INCLUDED
