#ifndef DELAUNAY_POINT_H_INCLUDED
#define DELAUNAY_POINT_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <vector>


class DelaunayPoint
{
    public:
        int m_idx{};
        std::pair<int, int> m_xy{};
        std::vector<int> m_connections{};

//        double angleToConnection(const int idxOfPt);
        void createEdge(const int idxOfPoint);
//        void deleteEdge(const int idxToDelete);
//        std::vector<DelaunayLine> getLines();

        DelaunayPoint& operator= (const DelaunayPoint &dpSource);

        DelaunayPoint(int idx, int x, int y): m_idx{idx}, m_xy{x, y} { /*Empty constructor*/ }
        DelaunayPoint(): m_idx{-1} { /*Empty constructor*/ }

        friend std::ostream& operator<< (std::ostream &out, DelaunayPoint &dp);
};


#endif // DELAUNAY_POINT_H_INCLUDED
