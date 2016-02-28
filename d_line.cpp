#include "d_line.h"


DLine& DLine::operator= (const DLine &dlSource)
{
    copySourceLine(dlSource);
    return *this;
}


void DLine::copySourceLine(const DLine &dlSource)
{
    m_idxL = dlSource.m_idxL;
    m_idxR = dlSource.m_idxR;
    m_dpL  = dlSource.m_dpL;
    m_dpR  = dlSource.m_dpR;
    m_xL   = dlSource.m_xL;
    m_xR   = dlSource.m_xR;
    m_yL   = dlSource.m_yL;
    m_yR   = dlSource.m_yR;
}


bool DLine::doesCrossLine(const DLine &otherLine) const
{
    // Calculated as described here, uses same variable names
    //   https://www.quora.com/Given-four-Cartesian-coordinates-how-do-I-check-whether-these-two-segments-intersect-or-not-using-C-C++
    Eigen::Vector3d AB{static_cast<double>(m_xR - m_xL),
                       static_cast<double>(m_yR - m_yL), 0};
    Eigen::Vector3d BC{static_cast<double>(otherLine.m_xL - m_xR),
                       static_cast<double>(otherLine.m_yL - m_yR), 0};
    Eigen::Vector3d BD{static_cast<double>(otherLine.m_xR - m_xR),
                       static_cast<double>(otherLine.m_yR - m_yR), 0};

    Eigen::Vector3d CD{static_cast<double>(otherLine.m_xR - otherLine.m_xL),
                       static_cast<double>(otherLine.m_yR - otherLine.m_yL), 0};
    Eigen::Vector3d DA{static_cast<double>(m_xL - otherLine.m_xR),
                       static_cast<double>(m_yL - otherLine.m_yR), 0};
    Eigen::Vector3d DB{static_cast<double>(m_xR - otherLine.m_xR),
                       static_cast<double>(m_yR - otherLine.m_yR), 0};

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

    Eigen::Vector2d A{static_cast<double>(m_xL),           static_cast<double>(m_yL)};
    Eigen::Vector2d B{static_cast<double>(m_xR),           static_cast<double>(m_yR)};
    Eigen::Vector2d C{static_cast<double>(otherLine.m_xL), static_cast<double>(otherLine.m_yL)};
    Eigen::Vector2d D{static_cast<double>(otherLine.m_xR), static_cast<double>(otherLine.m_yR)};

    if ( onSegment(C, A, B) || onSegment(D, A, B) || onSegment(A, C, D) || onSegment(B, C, D) )
        return true;

    return false;
}


int DLine::getLeftIdx() const { return m_idxL; }


int DLine::getRightIdx() const { return m_idxR; }


DPoint DLine::getLeftPoint() const { return m_dpL; }


DPoint DLine::getRightPoint() const { return m_dpR; }


std::ostream& operator<< (std::ostream &out, DLine &dl)
{
    out << "Line from (" << dl.m_xL   << ", " << dl.m_yL   << ") to " <<
                     "(" << dl.m_xR   << ", " << dl.m_yR   << "), with DPoints " <<
                     "[" << dl.m_idxL << ", " << dl.m_idxR << "]";
    return out;
}


bool valueIsBetween(int testValue, int side1, int side2)
{
    if ( (testValue <= side1 && testValue >= side2) ||
         (testValue <= side2 && testValue >= side1) )
         return true;
    return false;
}


bool onSegment(Eigen::Vector2d testPoint, Eigen::Vector2d side1, Eigen::Vector2d side2)
{
    double segmentLength = (side2 - side1).norm();
    double segmentLengthWithTestPoint = (testPoint - side1).norm() +
                                        (side2 - testPoint).norm();

    // Doesn't count sharing an endpoint as being "on the segment"
    if ( almostEqual( (testPoint - side1).norm(), 0.0 ) ||
         almostEqual( (side2 - testPoint).norm(), 0.0 ) )
    {
        return false;
    }

    return ( almostEqual(segmentLength, segmentLengthWithTestPoint) );
}
