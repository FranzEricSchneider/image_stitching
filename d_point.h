#ifndef D_POINT_H_INCLUDED
#define D_POINT_H_INCLUDED


#include <algorithm>
#include <iostream>
#include <vector>


class DPoint
{
    public:
        int m_idx{};
        int m_x{}, m_y{};
        std::vector<int> m_connections{};

        DPoint(int idx, int x, int y): m_idx{idx}, m_x{x}, m_y{y} { /*Empty constructor*/ }
        DPoint(): m_idx{-1} { /*Empty constructor*/ }

        void createEdge(const int idxOfPoint);
        void deleteEdge(const int idxToDelete);
        bool isConnected(const int idx);
        DPoint& operator= (const DPoint &dpSource);
        friend std::ostream& operator<< (std::ostream &out, DPoint &dp);
};


#endif // D_POINT_H_INCLUDED
