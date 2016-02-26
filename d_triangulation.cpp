#include "d_triangulation.h"


// Sorts the vector so the leftmost point is first in the set
bool vector3iComparison(Eigen::Vector3i lhs, Eigen::Vector3i rhs)
{
    if (lhs[0] == rhs[0])
        return lhs[1] < rhs[1];
    return lhs[0] < rhs[0];
}


DTriangulation& DTriangulation::operator= (const DTriangulation &dtSource)
{
    m_pointMap = dtSource.m_pointMap;
    return *this;
}


int DTriangulation::pointWithLowestY()
{
    std::map<int, DPoint>::iterator it = m_pointMap.begin();
    int idx{it->first};
    int lowestYVal{it->second.m_xy.second};

    for (it = m_pointMap.begin(); it!=m_pointMap.end(); ++it)
    {
        if ( it->second.m_xy.second < lowestYVal )
        {
            idx = it->first;
            lowestYVal = it->second.m_xy.second;
        }
    }

    return idx;
}


int DTriangulation::pointWithLowestYAboveGivenIdx(int givenIdx)
{
    std::map<int, DPoint>::iterator it = m_pointMap.begin();
    int idxToReturn{givenIdx};
    int lowestYVal{m_pointMap[givenIdx].m_xy.second};

    for (it = m_pointMap.begin(); it!=m_pointMap.end(); ++it)
    {
        int idxVal = it->first;
        int yVal = it->second.m_xy.second;
        if ( (idxVal != givenIdx && idxToReturn == givenIdx && yVal >= m_pointMap[givenIdx].m_xy.second) ||
             (idxVal != givenIdx && yVal < lowestYVal       && yVal >= m_pointMap[givenIdx].m_xy.second) )
        {
            idxToReturn = idxVal;
            lowestYVal = yVal;
        }
    }

    return idxToReturn;
}


void DTriangulation::triangulate()
{
    if (m_pointMap.size() <= 3)
    {
        completelyConnectSet();
    }
    else
    {
        std::map<int, DPoint> leftMap;
        std::map<int, DPoint> rightMap;

        int counter{};
        for (auto it = m_pointMap.begin(); it!=m_pointMap.end(); ++it)
        {
            if ( counter < (static_cast<int>(m_pointMap.size()) / 2) )
                leftMap.insert( std::pair<int, DPoint> (it->first, it->second) );
            else
                rightMap.insert( std::pair<int, DPoint> (it->first, it->second) );
            ++counter;
        }

        mergeGroups( DTriangulation{leftMap},
                     DTriangulation{rightMap} );
    }
}


void DTriangulation::completelyConnectSet()
{
    for (std::map<int, DPoint>::iterator outerIt = m_pointMap.begin();
         outerIt != --m_pointMap.end();
         ++outerIt)
    {
        ++outerIt;
        std::map<int, DPoint>::iterator innerIt{outerIt};
        --outerIt;
        for ( /*Already instantiated*/ ; innerIt != m_pointMap.end(); ++innerIt)
        {
            outerIt->second.createEdge(innerIt->first);
            innerIt->second.createEdge(outerIt->first);
        }
    }
}


// TODO: REREAD THE SECTION ON REFERENCES AND FIND OUT IF AND HOW TO MAKE THESE REFERENCES
void DTriangulation::mergeGroups(DTriangulation leftSide,
                                 DTriangulation rightSide)
{
    copyConnectionsToThisMap(leftSide);
    copyConnectionsToThisMap(rightSide);
    DLine firstLine = findFirstLine(leftSide, rightSide);
    m_pointMap[firstLine.m_idx1].createEdge(firstLine.m_idx2);
    m_pointMap[firstLine.m_idx2].createEdge(firstLine.m_idx1);

    bool hasLeftCandidate{true};
    bool hasRightCandidate{true};
    DLine baseLREdge = firstLine;

    while (hasLeftCandidate || hasRightCandidate)
    {
        DPoint leftCandidate  = getCandidate(baseLREdge, leftSide,  true);
        DPoint rightCandidate = getCandidate(baseLREdge, rightSide, false);

        hasLeftCandidate  = leftCandidate.m_idx  != -1;
        hasRightCandidate = rightCandidate.m_idx != -1;

        if (hasLeftCandidate && hasRightCandidate)
        {
            if ( circleContainsPoint(leftCandidate, baseLREdge, rightCandidate) )
                hasLeftCandidate = false;
            else
                hasRightCandidate = false;
        }

        if (hasLeftCandidate)
        {
            m_pointMap[leftCandidate.m_idx].createEdge(baseLREdge.getRightIdx());
            m_pointMap[baseLREdge.getRightIdx()].createEdge(leftCandidate.m_idx);
            baseLREdge = DLine{leftCandidate, baseLREdge.getRightPoint()};
        } else if (hasRightCandidate)
        {
            m_pointMap[rightCandidate.m_idx].createEdge(baseLREdge.getLeftIdx());
            m_pointMap[baseLREdge.getLeftIdx()].createEdge(rightCandidate.m_idx);
            baseLREdge = DLine{rightCandidate, baseLREdge.getLeftPoint()};
        }
    }
}


void DTriangulation::copyConnectionsToThisMap(DTriangulation subDT)
{
    for (std::map<int, DPoint>::iterator it = subDT.m_pointMap.begin();
         it != subDT.m_pointMap.end();
         ++it)
    {
        m_pointMap[it->first] = it->second;
    }
}


DLine DTriangulation::findFirstLine(DTriangulation &leftSide,
                                    DTriangulation &rightSide)
{
    std::vector<DLine> leftLines  = leftSide.getLines();
    std::vector<DLine> rightLines = rightSide.getLines();

    DLine firstLine;
    bool found{false};
    int leftPoint{-1}, rightPoint{-1};

    while (!found)
    {
        // Selects a line, starting with two lowest points and stepping up from there
        if (leftPoint == -1 && rightPoint == -1)
        {
            leftPoint  = leftSide.pointWithLowestY();
            rightPoint = rightSide.pointWithLowestY();
        }
        else
        {
            int originalValue;
            if ( leftSide.m_pointMap[leftPoint].m_xy.second <
                 rightSide.m_pointMap[rightPoint].m_xy.second )
            {
                originalValue = leftPoint;
                leftPoint = leftSide.pointWithLowestYAboveGivenIdx(leftPoint);
            } else
            {
                originalValue = rightPoint;
                rightPoint = rightSide.pointWithLowestYAboveGivenIdx(rightPoint);
            }

            if (originalValue == leftPoint || originalValue == rightPoint)
            {
                std::cout << "No first line was found!\nTerminating execution\n";
                exit(-1);
            }
        }

        bool leftSideLower = leftSide.m_pointMap[leftPoint].m_xy.second <
                             rightSide.m_pointMap[rightPoint].m_xy.second;
        if (leftSideLower)
            firstLine = getBaseEdge(leftSide.m_pointMap[leftPoint], rightSide, leftSideLower);
        else
            firstLine = getBaseEdge(rightSide.m_pointMap[rightPoint], leftSide, leftSideLower);

        // Checks that the line is legit
        found = true;
        for (auto line: leftSide.getLines())
        {
            if (found && firstLine.doesCrossLine(line))
                found = false;
        }
        for (auto line: rightSide.getLines())
        {
            if (found && firstLine.doesCrossLine(line))
                found = false;
        }
    }

    return firstLine;
}


// TODO: TRY TO MAKE dt CONST
DLine DTriangulation::getBaseEdge(const DPoint &point,
                                                DTriangulation &dt,
                                                bool pointIsOnLeft)
{
    std::set< std::pair<double, int>, sortFirstElementDescending > anglesFromPointSet;
    std::map<int, DPoint>::iterator mapIt = dt.m_pointMap.begin();
    for (mapIt = dt.m_pointMap.begin(); mapIt!=dt.m_pointMap.end(); ++mapIt)
    {
        Eigen::Vector2d vec{mapIt->second.m_xy.first - point.m_xy.first,
                            mapIt->second.m_xy.second - point.m_xy.second};
        vec.normalize();

        // If point is on the right side, flip the x values so that largest X will
        //   correspond with the most "outer" edge
        if (!pointIsOnLeft)
            vec[0] *= -1;

        std::pair<double, int> xValue{vec[0], mapIt->first};
        anglesFromPointSet.insert(xValue);
    }

    std::set< std::pair<double, int> >::iterator setIt = anglesFromPointSet.begin();
    return DLine{ point, dt.m_pointMap[(*setIt).second] };
}


DPoint DTriangulation::getCandidate(const DLine &line,
                                                  DTriangulation &dt,
                                                  bool isLeftCandidate)
{
    std::set< std::pair<double, int>, sortFirstElementAscending > anglesFromLineSet;
    if (isLeftCandidate)
        populateLeftCandidateSet(line, dt, anglesFromLineSet);
    else
        populateRightCandidateSet(line, dt, anglesFromLineSet);

    int coreIdx = isLeftCandidate?line.getLeftIdx():line.getRightIdx();
    std::set< std::pair<double, int> >::iterator setIt = anglesFromLineSet.begin();
    for (setIt = anglesFromLineSet.begin(); setIt != anglesFromLineSet.end(); /*Nothing*/ )
    {
        if ( (*setIt).first > PI )
            return DPoint{};  // Indicates no candidate was found

        int firstCandidateIdx = (*setIt).second;
        ++setIt;
        if (setIt == anglesFromLineSet.end())
            return dt.m_pointMap[firstCandidateIdx];  // Last viable point's automatically valid

        int secondCandidateIdx = (*setIt).second;
        if ( circleContainsPoint(dt.m_pointMap[firstCandidateIdx], line, dt.m_pointMap[secondCandidateIdx]) )
        {
            m_pointMap[firstCandidateIdx].deleteEdge(coreIdx);
            m_pointMap[coreIdx].deleteEdge(firstCandidateIdx);
            dt.m_pointMap[firstCandidateIdx].deleteEdge(coreIdx);
            dt.m_pointMap[coreIdx].deleteEdge(firstCandidateIdx);
        } else
        {
            return dt.m_pointMap[firstCandidateIdx];
        }
    }

    return DPoint{};  // Shouldn't ever get here, above cases should cover everything
}


void DTriangulation::populateLeftCandidateSet(const DLine &line,
                                                     DTriangulation &dt,
                                                     std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet)
{
    Eigen::Vector2i baseVector{line.getRightPoint().m_xy.first  - line.getLeftPoint().m_xy.first,
                               line.getRightPoint().m_xy.second - line.getLeftPoint().m_xy.second};
    for (int connectionIdx: dt.m_pointMap[line.getLeftIdx()].m_connections)
    {
        Eigen::Vector2i compareVector{dt.m_pointMap[connectionIdx].m_xy.first  - line.getLeftPoint().m_xy.first,
                                      dt.m_pointMap[connectionIdx].m_xy.second - line.getLeftPoint().m_xy.second};
        double angle = getCCWAngle(baseVector, compareVector);
        anglesFromLineSet.insert( std::pair<double, int>(angle, connectionIdx) );
    }
}


void DTriangulation::populateRightCandidateSet(const DLine &line,
                                                      DTriangulation &dt,
                                                      std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet)
{
    Eigen::Vector2i baseVector{line.getLeftPoint().m_xy.first  - line.getRightPoint().m_xy.first,
                               line.getLeftPoint().m_xy.second - line.getRightPoint().m_xy.second};
    for (int connectionIdx: dt.m_pointMap[line.getRightIdx()].m_connections)
    {
        Eigen::Vector2i compareVector{dt.m_pointMap[connectionIdx].m_xy.first  - line.getRightPoint().m_xy.first,
                                      dt.m_pointMap[connectionIdx].m_xy.second - line.getRightPoint().m_xy.second};
        double angle = getCWAngle(baseVector, compareVector);
        anglesFromLineSet.insert( std::pair<double, int>(angle, connectionIdx) );
    }
}


bool DTriangulation::circleContainsPoint(const DPoint &edgePoint,
                                                const DLine &edgeLine,
                                                const DPoint &innerPoint)
{
    double xCenter, yCenter, radius;
    calculateCircle(edgePoint, edgeLine, xCenter, yCenter, radius);
    double distToInnerPoint = sqrt( pow(xCenter - innerPoint.m_xy.first,  2) +
                                    pow(yCenter - innerPoint.m_xy.second, 2) );
    return distToInnerPoint <= radius;
}


double DTriangulation::getCCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison)
{
    double baseAngle = atan2(base[1], base[0]);
    double comparisonAngle = atan2(comparison[1], comparison[0]);

    if (comparisonAngle < baseAngle)
        comparisonAngle += 2 * PI;

    return comparisonAngle - baseAngle;
}


double DTriangulation::getCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison)
{
    double baseAngle = atan2(base[1], base[0]);
    double comparisonAngle = atan2(comparison[1], comparison[0]);

    if (comparisonAngle > baseAngle)
        comparisonAngle -= 2 * PI;

    return baseAngle - comparisonAngle;
}


// TODO: REWORK THIS LATER TO GET RID OF DOUBLED LINES
std::vector<DLine> DTriangulation::getLines()
{
    std::vector<DLine> lineVector;
    for (std::map<int, DPoint>::iterator it = m_pointMap.begin();
         it != m_pointMap.end();
         ++it)
    {
        for (int idx: it->second.m_connections)
        {
            lineVector.push_back( DLine{it->second, m_pointMap[idx]} );
        }
    }
    return lineVector;
}


std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > DTriangulation::getLinesForDrawingOrGraphing()
{
    std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > lineVector;
    for ( auto line: getLines() )
    {
        lineVector.push_back( std::pair<Eigen::Vector3i, Eigen::Vector3i>
                              ( Eigen::Vector3i(line.m_x1, line.m_y1, 0),
                                Eigen::Vector3i(line.m_x2, line.m_y2, 0) ) );
    }
    return lineVector;
}


void calculateCircle(const DPoint &point, const DLine &line,
                                            double &xCenter, double &yCenter, double &radius)
{
    // From here: http://paulbourke.net/geometry/circlesphere/
    //   variable names also taken from this paper
    double x1 = line.getLeftPoint().m_xy.first;
    double y1 = line.getLeftPoint().m_xy.second;
    double x2 = point.m_xy.first;
    double y2 = point.m_xy.second;
    double x3 = line.getRightPoint().m_xy.first;
    double y3 = line.getRightPoint().m_xy.second;
    if (x1 == x2)
    {
        std::swap(x2, x3);
        std::swap(y2, y3);
    }
    else if (x2 == x3)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    double mA = (y2 - y1) / (x2 - x1);
    double mB = (y3 - y2) / (x3 - x2);

    xCenter = ( mA * mB * (y1 - y3) + mB * (x1 + x2) - mA * (x2 + x3) ) / ( 2 * (mB - mA) );
    yCenter = (-1 / mA) * (xCenter - (x1 + x2) / 2) + (y1 + y2) / 2;
    radius = sqrt( pow(xCenter - x1, 2) + pow(yCenter - y1, 2) );
}
