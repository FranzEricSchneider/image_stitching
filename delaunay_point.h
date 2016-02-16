#ifndef DELAUNAY_POINT_H_INCLUDED
#define DELAUNAY_POINT_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <vector>


class DelaunayPoint
{
    public:
        int m_idx{};
        std::pair<double, double> m_xy{};
        std::vector<int> m_connections;

//        double angleToConnection(const int idxOfPt);
        void createEdge(const int idxOfPoint);
//        void deleteEdge(const int idxToDelete);
//        std::vector<DelaunayLine> getLines();

        DelaunayPoint& operator= (const DelaunayPoint &dpSource);

        DelaunayPoint(double x, double y): m_xy{x, y}
        {
        }

        DelaunayPoint()
        {
        }
};


#endif // DELAUNAY_POINT_H_INCLUDED
