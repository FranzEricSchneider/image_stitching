#ifndef DELAUNAY_LINE_H_INCLUDED
#define DELAUNAY_LINE_H_INCLUDED


#include <Eigen/Geometry>

#include "delaunay_point.h"
#include "simple_stitcher.h"


class DelaunayLine
{
    private:

    public:
        int m_idx1{}, m_idx2{};
        DelaunayPoint m_dp1{}, m_dp2{};

        // Make a line of the form (x2 - x1) / (y2 - y1), with defined x bounds
        int m_x1, m_x2, m_y1, m_y2;

        DelaunayLine& operator= (const DelaunayLine &dlSource);

        DelaunayLine() { /*Empty constructor*/ }

        DelaunayLine(int idx1, int idx2, DelaunayPoint dp1, DelaunayPoint dp2):
        m_idx1{idx1}, m_idx2{idx2}, m_dp1{dp1}, m_dp2{dp2}
        {
            // Deal with the vertical case first, simpler
            if (dp1.m_xy.first == dp2.m_xy.first)
            {
                m_x1 = m_x2 = dp1.m_xy.first;
                m_y1 = (dp1.m_xy.second < dp2.m_xy.second)?dp1.m_xy.second:dp2.m_xy.second;
                m_y2 = (dp1.m_xy.second > dp2.m_xy.second)?dp1.m_xy.second:dp2.m_xy.second;
            }
            else
            {
                if (dp1.m_xy.first < dp2.m_xy.first)
                {
                    m_x1 = dp1.m_xy.first;  m_y1 = dp1.m_xy.second;
                    m_x2 = dp2.m_xy.first;  m_y2 = dp2.m_xy.second;
                } else
                {
                    m_x2 = dp1.m_xy.first;  m_y2 = dp1.m_xy.second;
                    m_x1 = dp2.m_xy.first;  m_y1 = dp2.m_xy.second;
                }
            }
        }

        bool doesCrossLine(const DelaunayLine &otherLine);

// TODO: REREAD HOW TO OVERLOAD PRINT STATEMENT
};


bool valueIsBetween(int testValue, int side1, int side2);
bool onSegment(Eigen::Vector3d testPoint, Eigen::Vector3d side1, Eigen::Vector3d side2);


#endif // DELAUNAY_LINE_H_INCLUDED
