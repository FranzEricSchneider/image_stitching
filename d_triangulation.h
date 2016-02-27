#ifndef D_TRIANGULATION_H_INCLUDED
#define D_TRIANGULATION_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Geometry>
#include <vector>

#include "d_line.h"
#include "d_point.h"
#include "pair_comparison.h"


// TODO: FIND THE RIGHT WAY TO DEFINE PI. Eigen? std?
#define PI 3.141596


// Sorts the vector so the leftmost point is first in the set
bool vector3iComparison(Eigen::Vector3i lhs, Eigen::Vector3i rhs);


class DTriangulation
{
    private:
        void triangulate();
        void completelyConnectSet();
        void mergeGroups(DTriangulation leftSide, DTriangulation rightSide);
        void copyConnectionsToThisMap(DTriangulation subDT);
        DLine findFirstLine(DTriangulation &leftSide, DTriangulation &rightSide);
        DLine getBaseEdge(const DPoint &point, DTriangulation &dt, bool pointIsOnLeft);
        DPoint getCandidate(const DLine &line, DTriangulation &dt, bool isLeftCandidate);
        void populateLeftCandidateSet(const DLine &line, DTriangulation &dt,
                                      std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet);
        void populateRightCandidateSet(const DLine &line, DTriangulation &dt,
                                       std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet);
        bool circleContainsPoint(const DPoint &edgePoint, const DLine &edgeLine, const DPoint &innerPoint);
        double getCCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison);
        double getCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison);
        std::vector<DLine> getLines();

    public:
        std::map<int, DPoint> m_pointMap;

        DTriangulation& operator= (const DTriangulation &dtSource);

        DTriangulation() { /*Empty constructsor*/ }

        DTriangulation(std::map<int, DPoint> pointMap):
        m_pointMap{pointMap}
        {
            triangulate();
        }

        DTriangulation(std::vector<Eigen::Vector3i> baseSet)
        {
            std::sort(baseSet.begin(), baseSet.end(), vector3iComparison);
            int counter{};
            for (auto pt: baseSet)
            {
                DPoint x{counter, pt[0], pt[1]};
                m_pointMap.insert( std::pair<int, DPoint> (x.m_idx, x) );
                ++counter;
            }
            triangulate();
        }

        int pointWithLowestY();
        int pointWithLowestYAboveGivenIdx(int idx);
        std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > getLinesForDrawingOrGraphing();
};


void calculateCircle(const DPoint &point, const DLine &line, double &xCenter, double &yCenter, double &radius);


#endif // D_TRIANGULATION_H_INCLUDED
