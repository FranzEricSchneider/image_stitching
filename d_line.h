#ifndef D_LINE_H_INCLUDED
#define D_LINE_H_INCLUDED


#include <Eigen/Geometry>
#include <ostream>

#include "d_point.h"
#include "simple_stitcher.h"


class DLine
{
    private:

    public:
// TODO: CONSIDER MAKING IT LEFT AND RIGHT INSTEAD OF 1 AND 2
// TODO: CONSIDER MAKING SOME STUFF PRIVATE
        int m_idx1{}, m_idx2{};
        DPoint m_dp1{}, m_dp2{};

        // Make a line of the form (x2 - x1) / (y2 - y1), with defined x bounds
        int m_x1, m_x2, m_y1, m_y2;

        DLine& operator= (const DLine &dlSource);
        void copySourceLine(const DLine &dlSource);

        DLine() { /*Empty constructor*/ }

        DLine(const DLine &dlSource)
        {
            // Copy constructor
            copySourceLine(dlSource);
        }

        DLine(DPoint dp1, DPoint dp2):
        m_idx1{dp1.m_idx}, m_idx2{dp2.m_idx}, m_dp1{dp1}, m_dp2{dp2}
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

        bool doesCrossLine(const DLine &otherLine);
        int getLeftIdx() const;
        int getRightIdx() const;
        DPoint getLeftPoint() const;
        DPoint getRightPoint() const;
        friend std::ostream& operator<< (std::ostream &out, DLine &dl);
};


bool valueIsBetween(int testValue, int side1, int side2);
bool onSegment(Eigen::Vector3d testPoint, Eigen::Vector3d side1, Eigen::Vector3d side2);


#endif // D_LINE_H_INCLUDED
