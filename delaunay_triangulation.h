#ifndef DELAUNEY_TRIANGULATION_H_INCLUDED
#define DELAUNEY_TRIANGULATION_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "delaunay_point.h"


// Sorts the vector so the leftmost point is first in the set
bool vector3dComparison(Eigen::Vector3d lhs, Eigen::Vector3d rhs);


class DelaunayTriangulation
{
    private:
        const int m_numPoints;
        int pointWithLowestY();

        void triangulate();
        void completelyConnectSet();
        void mergeGroups(DelaunayTriangulation leftSide, DelaunayTriangulation rightSide );
        void copyConnectionsToThisMap(DelaunayTriangulation subDT);

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
                DelaunayPoint x{pt[0], pt[1]};
//                m_pointMap.insert( std::pair<int, DelaunayPoint> (counter, DelaunayPoint{pt[0], pt[1]}) );
                m_pointMap.insert( std::pair<int, DelaunayPoint> (counter, x) );
                ++counter;
            }
            triangulate();

            /*FOR DEBUGGING*/
            for (auto pt: baseSet)
            {
                std::cout << "baseSet point (x: " << pt[0] << ", y: " << pt[1] << ")\n";
            }
        }

};


#endif // DELAUNEY_TRIANGULATION_H_INCLUDED
