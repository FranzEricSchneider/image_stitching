#include "delaunay_line.h"


DelaunayLine& DelaunayLine::operator= (const DelaunayLine &dlSource)
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

    // Only taking the Z components of the cross product
    if ( (AB.cross(BC)[2] * AB.cross(BD)[2] < 0) &&
         (CD.cross(DA)[2] * AB.cross(DB)[2] < 0) )
        return true;

    Eigen::Vector3d A{m_x1, m_y1, 0};
    Eigen::Vector3d B{m_x2, m_y2, 0};
    Eigen::Vector3d C{otherLine.m_x1, otherLine.m_y1, 0};
    Eigen::Vector3d D{otherLine.m_x2, otherLine.m_y2, 0};

    if ( onSegment(C, A, B) || onSegment(D, A, B) || onSegment(A, C, D) || onSegment(B, C, D) )
        return true;

    return false;
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
    return ( almostEqual(lengthThroughTestPoint, segmentLength) );
}
