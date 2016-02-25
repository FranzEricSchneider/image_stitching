#include "delaunay_triangulation.h"


// Sorts the vector so the leftmost point is first in the set
bool vector3dComparison(Eigen::Vector3d lhs, Eigen::Vector3d rhs)
{
    if (lhs[0] == rhs[0])
        return lhs[1] < rhs[1];
    return lhs[0] < rhs[0];
}


int DelaunayTriangulation::pointWithLowestY()
{
    std::map<int, DelaunayPoint>::iterator it = m_pointMap.begin();
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


int DelaunayTriangulation::pointWithLowestYAboveGivenIdx(int givenIdx)
{
    std::map<int, DelaunayPoint>::iterator it = m_pointMap.begin();
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


void DelaunayTriangulation::triangulate()
{

    /*FOR DEBUGGING*/
//    std::map<int, DelaunayPoint>::iterator mapIt = m_pointMap.begin();
//    for (mapIt = m_pointMap.begin(); mapIt!=m_pointMap.end(); ++mapIt)
//    {
//        std::cout << "Map point: " << mapIt->first << " => (" << mapIt->second.m_xy.first << ", " << mapIt->second.m_xy.second << ")\n";
//        std::cout << "\tconnections: ";
//        for (auto connection: mapIt->second.m_connections)
//        {
//            std::cout << connection << ", ";
//        }
//        std::cout << "\n";
//    }
//    std::cout << "Point with lowest Y (idx): " << pointWithLowestY() << "\n";
//    std::cout << "\n";

    if (m_numPoints <= 3)
    {
        completelyConnectSet();
    }
    else
    {
        std::map<int, DelaunayPoint> leftMap;
        std::map<int, DelaunayPoint> rightMap;

        int counter{};
        for (auto it = m_pointMap.begin(); it!=m_pointMap.end(); ++it)
        {
            if ( counter < (m_numPoints / 2) )
                leftMap.insert( std::pair<int, DelaunayPoint> (it->first, it->second) );
            else
                rightMap.insert( std::pair<int, DelaunayPoint> (it->first, it->second) );
            ++counter;
        }

        mergeGroups( DelaunayTriangulation{leftMap},
                     DelaunayTriangulation{rightMap} );
    }
}


void DelaunayTriangulation::completelyConnectSet()
{
    for (std::map<int, DelaunayPoint>::iterator outerIt = m_pointMap.begin();
         outerIt != --m_pointMap.end();
         ++outerIt)
    {
        ++outerIt;
        std::map<int, DelaunayPoint>::iterator innerIt{outerIt};
        --outerIt;
        for ( /*Already instantiated*/ ; innerIt != m_pointMap.end(); ++innerIt)
        {
            outerIt->second.createEdge(innerIt->first);
            innerIt->second.createEdge(outerIt->first);
        }
    }
}


// TODO: REREAD THE SECTION ON REFERENCES AND FIND OUT IF AND HOW TO MAKE THESE REFERENCES
void DelaunayTriangulation::mergeGroups( DelaunayTriangulation leftSide,
                                         DelaunayTriangulation rightSide )
{
    copyConnectionsToThisMap(leftSide);
    copyConnectionsToThisMap(rightSide);
    DelaunayLine firstLine = findFirstLine(leftSide, rightSide);
    m_pointMap[firstLine.m_idx1].createEdge(firstLine.m_idx2);
    m_pointMap[firstLine.m_idx2].createEdge(firstLine.m_idx1);

    bool hasLeftCandidate{true};
    bool hasRightCandidate{true};
    DelaunayLine baseLREdge = firstLine;

    while (hasLeftCandidate || hasRightCandidate)
    {
        DelaunayPoint leftCandidate  = getCandidate(baseLREdge, leftSide,  true);
        DelaunayPoint rightCandidate = getCandidate(baseLREdge, rightSide, false);

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
            baseLREdge = DelaunayLine{leftCandidate, baseLREdge.getRightPoint()};
        } else if (hasRightCandidate)
        {
            m_pointMap[rightCandidate.m_idx].createEdge(baseLREdge.getLeftIdx());
            m_pointMap[baseLREdge.getLeftIdx()].createEdge(rightCandidate.m_idx);
            baseLREdge = DelaunayLine{rightCandidate, baseLREdge.getLeftPoint()};
        }
    }
}


void DelaunayTriangulation::copyConnectionsToThisMap(DelaunayTriangulation subDT)
{
    for (std::map<int, DelaunayPoint>::iterator it = subDT.m_pointMap.begin();
         it != subDT.m_pointMap.end();
         ++it)
    {
        m_pointMap[it->first] = it->second;
    }
}


DelaunayLine DelaunayTriangulation::findFirstLine(DelaunayTriangulation &leftSide,
                                                  DelaunayTriangulation &rightSide)
{
    std::vector<DelaunayLine> leftLines = leftSide.getLines();
    std::vector<DelaunayLine> rightLines = rightSide.getLines();

    DelaunayLine firstLine;
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
DelaunayLine DelaunayTriangulation::getBaseEdge(const DelaunayPoint &point,
                                                DelaunayTriangulation &dt,
                                                bool pointIsOnLeft)
{
    std::set< std::pair<double, int>, sortFirstElementDescending > anglesFromPointSet;
    std::map<int, DelaunayPoint>::iterator mapIt = dt.m_pointMap.begin();
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
    return DelaunayLine{ point, dt.m_pointMap[(*setIt).second] };
}


DelaunayPoint DelaunayTriangulation::getCandidate(const DelaunayLine &line,
                                                  DelaunayTriangulation &dt,
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
            return DelaunayPoint{};  // Indicates no candidate was found

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

    return DelaunayPoint{};  // Shouldn't ever get here, above cases should cover everything
}


void DelaunayTriangulation::populateLeftCandidateSet(const DelaunayLine &line,
                                                     DelaunayTriangulation &dt,
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


void DelaunayTriangulation::populateRightCandidateSet(const DelaunayLine &line,
                                                      DelaunayTriangulation &dt,
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


bool DelaunayTriangulation::circleContainsPoint(const DelaunayPoint &edgePoint,
                                                const DelaunayLine &edgeLine,
                                                const DelaunayPoint &innerPoint)
{
    double xCenter, yCenter, radius;
    calculateCircle(edgePoint, edgeLine, xCenter, yCenter, radius);
    double distToInnerPoint = sqrt( pow(xCenter - innerPoint.m_xy.first,  2) +
                                    pow(yCenter - innerPoint.m_xy.second, 2) );
    return distToInnerPoint <= radius;
}


double DelaunayTriangulation::getCCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison)
{
    double baseAngle = atan2(base[1], base[0]);
    double comparisonAngle = atan2(comparison[1], comparison[0]);

    if (comparisonAngle < baseAngle)
        comparisonAngle += 2 * PI;

    return comparisonAngle - baseAngle;
}


double DelaunayTriangulation::getCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison)
{
    double baseAngle = atan2(base[1], base[0]);
    double comparisonAngle = atan2(comparison[1], comparison[0]);

    if (comparisonAngle > baseAngle)
        comparisonAngle -= 2 * PI;

    return baseAngle - comparisonAngle;
}


// TODO: REWORK THIS LATER TO GET RID OF DOUBLED LINES
std::vector<DelaunayLine> DelaunayTriangulation::getLines()
{
    std::vector<DelaunayLine> lineVector;
    for (std::map<int, DelaunayPoint>::iterator it = m_pointMap.begin();
         it != m_pointMap.end();
         ++it)
    {
        for (int idx: it->second.m_connections)
        {
            lineVector.push_back( DelaunayLine{it->second, m_pointMap[idx]} );
        }
    }
    return lineVector;
}


std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > DelaunayTriangulation::getLinesForDrawingOrGraphing()
{
    std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > lineVector;
    for ( auto line: getLines() )
    {
        lineVector.push_back( std::pair<Eigen::Vector3d, Eigen::Vector3d>
                              ( Eigen::Vector3d(line.m_x1, line.m_y1, 0),
                                Eigen::Vector3d(line.m_x2, line.m_y2, 0) ) );
    }
    return lineVector;
}


void calculateCircle(const DelaunayPoint &point, const DelaunayLine &line,
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
