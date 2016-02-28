#ifndef D_LINE_H_INCLUDED
#define D_LINE_H_INCLUDED


#include <Eigen/Geometry>
#include <ostream>

#include "d_point.h"
#include "simple_stitcher.h"


class DLine
{
    private:
        int m_idxL{}, m_idxR{};
        DPoint m_dpL{}, m_dpR{};

    public:
// TODO: CONSIDER MAKING SOME STUFF PRIVATE

        // Makes a line of the form (xR - xL) / (yR - yL)
        int m_xL, m_xR, m_yL, m_yR;

        DLine& operator= (const DLine &dlSource);
        void copySourceLine(const DLine &dlSource);

        DLine() { /*Empty constructor*/ }

        DLine(const DLine &dlSource)
        {
            // Copy constructor
            copySourceLine(dlSource);
        }

        DLine(DPoint dp1, DPoint dp2)
        {
            if ( (dp1.m_x == dp2.m_x && dp1.m_y < dp2.m_y) || (dp1.m_x < dp2.m_x) )
            {
                m_xL   = dp1.m_x;   m_xR   = dp2.m_x;
                m_yL   = dp1.m_y;   m_yR   = dp2.m_y;
                m_idxL = dp1.m_idx; m_idxR = dp2.m_idx;
                m_dpL  = dp1;       m_dpR  = dp2;
            } else
            {
                m_xL   = dp2.m_x;   m_xR   = dp1.m_x;
                m_yL   = dp2.m_y;   m_yR   = dp1.m_y;
                m_idxL = dp2.m_idx; m_idxR = dp1.m_idx;
                m_dpL  = dp2;       m_dpR  = dp1;
            }
        }

        bool doesCrossLine(const DLine &otherLine) const;
        int getLeftIdx() const;
        int getRightIdx() const;
        DPoint getLeftPoint() const;
        DPoint getRightPoint() const;
        friend std::ostream& operator<< (std::ostream &out, DLine &dl);
};


bool valueIsBetween(int testValue, int side1, int side2);
bool onSegment(Eigen::Vector2d testPoint, Eigen::Vector2d side1, Eigen::Vector2d side2);


#endif // D_LINE_H_INCLUDED
