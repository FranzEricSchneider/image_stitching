#ifndef D_POINT_H_INCLUDED
#define D_POINT_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <vector>


class DPoint
{
    public:
        int m_idx{};
        std::pair<int, int> m_xy{};
        std::vector<int> m_connections{};

//        double angleToConnection(const int idxOfPt);
        void createEdge(const int idxOfPoint);
        void deleteEdge(const int idxToDelete);
        bool isConnected(const int idx);
//        std::vector<DLine> getLines();

        DPoint& operator= (const DPoint &dpSource);

        DPoint(int idx, int x, int y): m_idx{idx}, m_xy{x, y} { /*Empty constructor*/ }
        DPoint(): m_idx{-1} { /*Empty constructor*/ }

        friend std::ostream& operator<< (std::ostream &out, DPoint &dp);
};


#endif // D_POINT_H_INCLUDED
