#include "d_line.h"


DLine& DLine::operator= (const DLine &dlSource)
{
    copySourceLine(dlSource);
    return *this;
}


void DLine::copySourceLine(const DLine &dlSource)
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


bool DLine::doesCrossLine(const DLine &otherLine)
{
    // Calculated as described here: https://www.quora.com/Given-four-Cartesian-coordinates-how-do-I-check-whether-these-two-segments-intersect-or-not-using-C-C++
    Eigen::Vector3d AB{static_cast<double>(m_x2 - m_x1),
                       static_cast<double>(m_y2 - m_y1), 0};
    Eigen::Vector3d BC{static_cast<double>(otherLine.m_x1 - m_x2),
                       static_cast<double>(otherLine.m_y1 - m_y2), 0};
    Eigen::Vector3d BD{static_cast<double>(otherLine.m_x2 - m_x2),
                       static_cast<double>(otherLine.m_y2 - m_y2), 0};

    Eigen::Vector3d CD{static_cast<double>(otherLine.m_x2 - otherLine.m_x1),
                       static_cast<double>(otherLine.m_y2 - otherLine.m_y1), 0};
    Eigen::Vector3d DA{static_cast<double>(m_x1 - otherLine.m_x2),
                       static_cast<double>(m_y1 - otherLine.m_y2), 0};
    Eigen::Vector3d DB{static_cast<double>(m_x2 - otherLine.m_x2),
                       static_cast<double>(m_y2 - otherLine.m_y2), 0};

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

    Eigen::Vector3d A{static_cast<double>(m_x1), static_cast<double>(m_y1), 0};
    Eigen::Vector3d B{static_cast<double>(m_x2), static_cast<double>(m_y2), 0};
    Eigen::Vector3d C{static_cast<double>(otherLine.m_x1), static_cast<double>(otherLine.m_y1), 0};
    Eigen::Vector3d D{static_cast<double>(otherLine.m_x2), static_cast<double>(otherLine.m_y2), 0};

    if ( onSegment(C, A, B) || onSegment(D, A, B) || onSegment(A, C, D) || onSegment(B, C, D) )
        return true;

    return false;
}


int DLine::getLeftIdx() const
{
    if (m_dp1.m_xy.first == m_dp2.m_xy.first)
        return (m_dp1.m_xy.second < m_dp2.m_xy.second)?m_idx1:m_idx2;
    return (m_dp1.m_xy.first < m_dp2.m_xy.first)?m_idx1:m_idx2;
}


int DLine::getRightIdx() const
{
    return (getLeftIdx() == m_idx1)?m_idx2:m_idx1;
}


DPoint DLine::getLeftPoint() const
{
    return (getLeftIdx() == m_idx1)?m_dp1:m_dp2;
}


DPoint DLine::getRightPoint() const
{
    return (getLeftIdx() == m_idx1)?m_dp2:m_dp1;
}


std::ostream& operator<< (std::ostream &out, DLine &dl)
{
    out << "Line from (" << dl.m_x1   << ", " << dl.m_y1   << ") to " <<
                     "(" << dl.m_x2   << ", " << dl.m_y2   << "), with DPoints " <<
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


bool onSegment(Eigen::Vector3d testPoint, Eigen::Vector3d side1, Eigen::Vector3d side2)
{
    double lengthThroughTestPoint = (testPoint - side1).norm() +
                                    (side2 - testPoint).norm();
    double segmentLength = (side2 - side1).norm();

    // Doesn't count sharing an endpoint as being "on the segment"
    if ( almostEqual( (testPoint - side1).norm(), 0.0 ) ||
         almostEqual( (side2 - testPoint).norm(), 0.0 ) )
    {
        return false;
    }

    return ( almostEqual(lengthThroughTestPoint, segmentLength) );
}
