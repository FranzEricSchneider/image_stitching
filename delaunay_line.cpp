#include "delaunay_line.h"


DelaunayLine& DelaunayLine::operator= (const DelaunayLine &dlSource)
{
    copySourceLine(dlSource);
    return *this;
}


void DelaunayLine::copySourceLine(const DelaunayLine &dlSource)
{
    m_idx1 = dlSource.m_idx1;
    m_idx2 = dlSource.m_idx2;
    m_dp1 = dlSource.m_dp1;
    m_dp2 = dlSource.m_dp2;
    m_x1 = dlSource.m_x1;
    m_x2 = dlSource.m_x2;
    m_y1 = dlSource.m_y1;
    m_y2 = dlSource.m_y2;
}


bool DelaunayLine::doesCrossLine(const DelaunayLine &otherLine)
{
    // Calculated as described here: https://www.quora.com/Given-four-Cartesian-coordinates-how-do-I-check-whether-these-two-segments-intersect-or-not-using-C-C++
    Eigen::Vector3d AB{m_x2 - m_x1, m_y2 - m_y1, 0};
    Eigen::Vector3d BC{otherLine.m_x1 - m_x2, otherLine.m_y1 - m_y2, 0};
    Eigen::Vector3d BD{otherLine.m_x2 - m_x2, otherLine.m_y2 - m_y2, 0};

    Eigen::Vector3d CD{otherLine.m_x2 - otherLine.m_x1, otherLine.m_y2 - otherLine.m_y1, 0};
    Eigen::Vector3d DA{m_x1 - otherLine.m_x2, m_y1 - otherLine.m_y2, 0};
    Eigen::Vector3d DB{m_x2 - otherLine.m_x2, m_y2 - otherLine.m_y2, 0};

    Eigen::Vector3d unitAB = AB / AB.norm();
    Eigen::Vector3d unitBC = BC / BC.norm();
    Eigen::Vector3d unitBD = BD / BD.norm();

    Eigen::Vector3d unitCD = CD / CD.norm();
    Eigen::Vector3d unitDA = DA / DA.norm();
    Eigen::Vector3d unitDB = DB / DB.norm();

    // Only taking the Z components of the cross product
    if ( (unitAB.cross(unitBC)[2] * unitAB.cross(unitBD)[2] < 0) &&
         (unitCD.cross(unitDA)[2] * unitCD.cross(unitDB)[2] < 0) )
        return true;

    Eigen::Vector3i A{m_x1, m_y1, 0};
    Eigen::Vector3i B{m_x2, m_y2, 0};
    Eigen::Vector3i C{otherLine.m_x1, otherLine.m_y1, 0};
    Eigen::Vector3i D{otherLine.m_x2, otherLine.m_y2, 0};

    if ( onSegment(C, A, B) || onSegment(D, A, B) || onSegment(A, C, D) || onSegment(B, C, D) )
        return true;

    return false;
}


int DelaunayLine::getLeftIdx() const
{
    if (m_dp1.m_xy.first == m_dp2.m_xy.first)
        return (m_dp1.m_xy.second < m_dp2.m_xy.second)?m_idx1:m_idx2;
    return (m_dp1.m_xy.first < m_dp2.m_xy.first)?m_idx1:m_idx2;
}


int DelaunayLine::getRightIdx() const
{
    return (getLeftIdx() == m_idx1)?m_idx2:m_idx1;
}


DelaunayPoint DelaunayLine::getLeftPoint() const
{
    return (getLeftIdx() == m_idx1)?m_dp1:m_dp2;
}


DelaunayPoint DelaunayLine::getRightPoint() const
{
    return (getLeftIdx() == m_idx1)?m_dp2:m_dp1;
}


std::ostream& operator<< (std::ostream &out, DelaunayLine &dl)
{
    out << "Line from (" << dl.m_x1   << ", " << dl.m_y1   << ") to " <<
                     "(" << dl.m_x2   << ", " << dl.m_y2   << "), with DelaunayPoints " <<
                     "[" << dl.m_idx1 << ", " << dl.m_idx2 << "]";
    return out;
}


bool valueIsBetween(int testValue, int side1, int side2)
{
    if ( (testValue <= side1 && testValue >= side2) ||
         (testValue <= side2 && testValue >= side1) )
         return true;
    return false;
}


bool onSegment(Eigen::Vector3i testPoint, Eigen::Vector3i side1, Eigen::Vector3i side2)
{
    double lengthThroughTestPoint = (testPoint - side1).norm() +
                                    (side2 - testPoint).norm();
    double segmentLength = (side2 - side1).norm();

    // Does count sharing an endpoint as being "on the segment"
    if ( almostEqual( (testPoint - side1).norm(), 0.0 ) ||
         almostEqual( (side2 - testPoint).norm(), 0.0 ) )
    {
        return false;
    }

    return ( almostEqual(lengthThroughTestPoint, segmentLength) );
}
