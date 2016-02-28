#ifndef D_TRIANGULATION_H_INCLUDED
#define D_TRIANGULATION_H_INCLUDED


#include <algorithm>
#include <Eigen/Geometry>
#include <iostream>
#include <map>
#include <vector>


#include "d_line.h"
#include "d_point.h"
#include "pair_comparison.h"


#define PI 3.141596


// Sorts the vector so the leftmost point is first in the set
bool vector3iComparison(const Eigen::Vector3i lhs, const Eigen::Vector3i rhs);


class DTriangulation
{
    private:
        void triangulate();
        void completelyConnectSet();
        void mergeGroups(DTriangulation leftSide, DTriangulation rightSide);
        void copyConnectionsToThisMap(const DTriangulation &subDT);
        DLine findFirstLine(const DTriangulation &leftSide, const DTriangulation &rightSide);
        int pointWithLowestY() const;
        int pointWithLowestYAboveGivenIdx(int idx) const;
        DLine getBaseEdge(const DPoint &point, const DTriangulation &dt, bool pointIsOnLeft);
        DPoint getCandidate(const DLine &line, DTriangulation &dt, bool isLeftCandidate);
        void populateLeftCandidateSet(const DLine &line, DTriangulation &dt,
                                      std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet);
        void populateRightCandidateSet(const DLine &line, DTriangulation &dt,
                                       std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet);
        bool circleContainsPoint(const DPoint &edgePoint, const DLine &edgeLine, const DPoint &innerPoint);
        double getCCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison);
        double getCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison);
        std::vector<DLine> getLines() const;

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

        std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > getLinesForDrawingOrGraphing();
};


void calculateCircle(const DPoint &point, const DLine &line, double &xCenter, double &yCenter, double &radius);


#endif // D_TRIANGULATION_H_INCLUDED
