#ifndef DELAUNAY_POINT_H_INCLUDED
#define DELAUNAY_POINT_H_INCLUDED


#include <vector>


class DelaunayPoint
{
    private:
        std::vector<DelaunayPoint*> connections;

    public:
        int m_idx;
        const std::pair<double, double> m_xy;

        double angleToConnection(const int idxOfPt);
        void deleteEdge(const int idxToDelete);
//        std::vector<DelaunayLine> getLines();

        DelaunayPoint(double x, double y): m_xy{x, y}
        {
        }

};


#endif // DELAUNAY_POINT_H_INCLUDED
