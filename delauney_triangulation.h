#ifndef DELAUNEY_TRIANGULATION_H_INCLUDED
#define DELAUNEY_TRIANGULATION_H_INCLUDED


#include <set>
#include <vector>

#include <Eigen/Geometry>

#include "delaunay_point.h"


// Sorts the set so the leftmost point is first in the set
struct delaunayPointComparison
{
    bool operator() (DelaunayPoint lhs, DelaunayPoint rhs) const
        { return lhs.m_xy.first < rhs.m_xy.first; }
};


class DelaunayTriangulation
{
    private:
        int pointWithLowestY();

    public:
        std::set<DelaunayPoint, delaunayPointComparison> m_pointSet;
//        std::set<DelaunayLine, delaunayLineComparison> m_lineSet;

        DelaunayTriangulation(std::vector<Eigen::Vector3d> baseSet)
        {
            for (auto pt: baseSet)
            {
                m_pointSet.insert(DelaunayPoint{pt[0], pt[1]});
            }
        }

};


#endif // DELAUNEY_TRIANGULATION_H_INCLUDED
