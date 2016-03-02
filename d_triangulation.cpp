#include "d_triangulation.h"


/* Sorts the vector so the left/bottom-most point is first in the set */
bool vector3iComparison(const Eigen::Vector3i lhs, const Eigen::Vector3i rhs)
{
    if (lhs[0] == rhs[0])
        return lhs[1] < rhs[1];
    return lhs[0] < rhs[0];
}


/*
If the set has three points or less, they become completely connected and the
function returns. Otherwise the set is split in two and mergeGroups is called
on the two smaller Delaunay Triangulation sets
*/
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
        for (std::map<int, DPoint>::iterator it = m_pointMap.begin();
             it!=m_pointMap.end();
             ++it)
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


/*
Edges are created between all available points. Two points will become a line, three
points will become a triangle
*/
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
            createEdgeInMap(outerIt->first, innerIt->first);
        }
    }
}


/*
As laid out in the paper, mergeGroups takes two sets of points, looks for a base line
(the bottom/outermost line between the left and right sets) then follows the merge algorithm
between the two sets. The merge continues drawing triangles between the groups until no
more legal triangles are available
*/
// TODO: REREAD THE SECTION ON REFERENCES AND FIND OUT IF AND HOW TO MAKE THESE REFERENCES
void DTriangulation::mergeGroups(DTriangulation leftSide,
                                 DTriangulation rightSide)
{
    copyConnectionsToThisMap(leftSide);
    copyConnectionsToThisMap(rightSide);
    DLine firstLine = findFirstLine(leftSide, rightSide);
    createEdgeInMap(firstLine.getLeftIdx(), firstLine.getRightIdx());

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
            createEdgeInMap(leftCandidate.m_idx, baseLREdge.getRightIdx());
            baseLREdge = DLine{leftCandidate, baseLREdge.getRightPoint()};
        } else if (hasRightCandidate)
        {
            createEdgeInMap(rightCandidate.m_idx, baseLREdge.getLeftIdx());
            baseLREdge = DLine{rightCandidate, baseLREdge.getLeftPoint()};
        }
    }
}


/*
Takes a different Delaunay Triangulation and copies its points into the m_pointMap
of this object. This is done so that smaller Delaunay Triangulations can be merged
back into a single Triangulation
*/
void DTriangulation::copyConnectionsToThisMap(const DTriangulation &subDT)
{
    for (std::map<int, DPoint>::const_iterator it = subDT.m_pointMap.begin();
         it != subDT.m_pointMap.end();
         ++it)
    {
        m_pointMap[it->first] = it->second;
    }
}


/*
Finds the bottom/outermost line between left and right sets to act as the
base edge for mergeGroups. This is done by
  1) Taking the lowest point between the two groups
  2) Finding outermost line between that point, all points in other group
  3) Checking that this outmost line didn't cross any existing lines
  4) If it crossed a line, take the next highest point
*/
DLine DTriangulation::findFirstLine(const DTriangulation &leftSide,
                                    const DTriangulation &rightSide)
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
            if ( leftSide.m_pointMap.at(leftPoint).m_y <
                 rightSide.m_pointMap.at(rightPoint).m_y )
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

        // Gets the outermost line between the lowest point and the other side
        bool leftSideLower = leftSide.m_pointMap.at(leftPoint).m_y <
                             rightSide.m_pointMap.at(rightPoint).m_y;
        if (leftSideLower)
            firstLine = getBaseEdge(leftSide.m_pointMap.at(leftPoint), rightSide, leftSideLower);
        else
            firstLine = getBaseEdge(rightSide.m_pointMap.at(rightPoint), leftSide, leftSideLower);

        // Checks that the found line doesn't cross any existing lines
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


/* Returns index of point with the lowest Y value */
int DTriangulation::pointWithLowestY() const
{
    std::map<int, DPoint>::const_iterator it = m_pointMap.begin();
    int idxToReturn{it->first};
    int lowestYVal{it->second.m_y};

    for (it = m_pointMap.begin(); it!=m_pointMap.end(); ++it)
    {
        if ( it->second.m_y < lowestYVal )
        {
            idxToReturn = it->first;
            lowestYVal = it->second.m_y;
        }
    }

    return idxToReturn;
}


/*
Returns index of point with the lowest Y value which is above the Y value
of the given index
*/
int DTriangulation::pointWithLowestYAboveGivenIdx(int givenIdx) const
{
    std::map<int, DPoint>::const_iterator it = m_pointMap.begin();
    int idxToReturn{givenIdx};
    int lowestYVal{m_pointMap.at(givenIdx).m_y};

    for (it = m_pointMap.begin(); it!=m_pointMap.end(); ++it)
    {
        int idxVal = it->first;
        int yVal = it->second.m_y;
        if ( (idxVal != givenIdx && idxToReturn == givenIdx && yVal >= m_pointMap.at(givenIdx).m_y) ||
             (idxVal != givenIdx && yVal < lowestYVal       && yVal >= m_pointMap.at(givenIdx).m_y) )
        {
            idxToReturn = idxVal;
            lowestYVal = yVal;
        }
    }

    return idxToReturn;
}


/*
Finds the outermost line between point and dt by making a set that sorts the
point indices by the x value of a normalized vector between point and each point
in dt. The normalized vectors with the largest x value are those closest to level
*/
DLine DTriangulation::getBaseEdge(const DPoint &point,
                                  const DTriangulation &dt,
                                  bool pointIsOnLeft)
{
    std::set< std::pair<double, int>, sortFirstElementDescending > anglesFromPointSet;
    for ( std::map<int, DPoint>::const_iterator mapIt = dt.m_pointMap.begin();
          mapIt!=dt.m_pointMap.end();
          ++mapIt )
    {
        Eigen::Vector2d vec{mapIt->second.m_x - point.m_x,
                            mapIt->second.m_y - point.m_y};
        vec.normalize();

        // If point is on the right side, flip the x values so that largest X will
        //   correspond with the most "outer" edge
        if (!pointIsOnLeft)
            vec[0] *= -1;

        anglesFromPointSet.insert( std::pair<double, int>(vec[0], mapIt->first) );
    }

    std::set< std::pair<double, int> >::iterator setIt = anglesFromPointSet.begin();
    return DLine{ point, dt.m_pointMap.at((*setIt).second) };
}


/*
As laid out in the paper, candidates are found that
  1) Are less the 180 degrees off from the given base edge (line)
  2) Do not contain the next candidate in their circumcircle
*/
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

    for ( std::set< std::pair<double, int> >::iterator setIt = anglesFromLineSet.begin();
          setIt != anglesFromLineSet.end();
          /*Nothing*/ )
    {
        if ( (*setIt).first > PI )
            return DPoint{};  // Indicates no candidate was found

        int firstCandidateIdx = (*setIt).second;
        ++setIt;
        if (setIt == anglesFromLineSet.end())
            return dt.m_pointMap[firstCandidateIdx];  // Last viable point is automatically valid

        int secondCandidateIdx = (*setIt).second;
        if ( circleContainsPoint(dt.m_pointMap[firstCandidateIdx], line, dt.m_pointMap[secondCandidateIdx]) )
        {
            deleteEdgeInMap(firstCandidateIdx, coreIdx);
            dt.deleteEdgeInMap(firstCandidateIdx, coreIdx);
        } else
        {
            return dt.m_pointMap[firstCandidateIdx];
        }
    }

    return DPoint{};  // Shouldn't ever get here, above cases should cover everything
}


/*
Both the left and right candidate sets are made so that the point indices are sorted such
that the point which is the least angle from the base line is first
*/
void DTriangulation::populateLeftCandidateSet(const DLine &line,
                                              DTriangulation &dt,
                                              std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet)
{
    Eigen::Vector2i baseVector{ line.getRightPoint().m_x - line.getLeftPoint().m_x,
                                line.getRightPoint().m_y - line.getLeftPoint().m_y };
    for (int connectionIdx: dt.m_pointMap[line.getLeftIdx()].m_connections)
    {
        Eigen::Vector2i compareVector{dt.m_pointMap[connectionIdx].m_x  - line.getLeftPoint().m_x,
                                      dt.m_pointMap[connectionIdx].m_y - line.getLeftPoint().m_y};
        double angle = getCCWAngle(baseVector, compareVector);
        anglesFromLineSet.insert( std::pair<double, int>(angle, connectionIdx) );
    }
}


/*
Both the left and right candidate sets are made so that the point indices are sorted such
that the point which is the least angle from the base line is first
*/
void DTriangulation::populateRightCandidateSet(const DLine &line,
                                               DTriangulation &dt,
                                               std::set< std::pair<double, int>, sortFirstElementAscending > &anglesFromLineSet)
{
    Eigen::Vector2i baseVector{ line.getLeftPoint().m_x - line.getRightPoint().m_x,
                                line.getLeftPoint().m_y - line.getRightPoint().m_y };
    for (int connectionIdx: dt.m_pointMap[line.getRightIdx()].m_connections)
    {
        Eigen::Vector2i compareVector{dt.m_pointMap[connectionIdx].m_x  - line.getRightPoint().m_x,
                                      dt.m_pointMap[connectionIdx].m_y - line.getRightPoint().m_y};
        double angle = getCWAngle(baseVector, compareVector);
        anglesFromLineSet.insert( std::pair<double, int>(angle, connectionIdx) );
    }
}


/* Checks whether a point is in/out of a circle defined by a line and a point */
bool DTriangulation::circleContainsPoint(const DPoint &edgePoint,
                                         const DLine  &edgeLine,
                                         const DPoint &innerPoint)
{
    double xCenter, yCenter, radius;
    calculateCircle(edgePoint, edgeLine, xCenter, yCenter, radius);
    double distToInnerPoint = sqrt( pow(xCenter - innerPoint.m_x, 2) +
                                    pow(yCenter - innerPoint.m_y, 2) );
    return distToInnerPoint <= radius;
}


/* Gets angle from base DLine to comparison DLine, going CCW */
double DTriangulation::getCCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison)
{
    double baseAngle = atan2(base[1], base[0]);
    double comparisonAngle = atan2(comparison[1], comparison[0]);

    if (comparisonAngle < baseAngle)
        comparisonAngle += 2 * PI;

    return comparisonAngle - baseAngle;
}


/* Gets angle from base DLine to comparison DLine, going CW */
double DTriangulation::getCWAngle(const Eigen::Vector2i &base, const Eigen::Vector2i &comparison)
{
    double baseAngle = atan2(base[1], base[0]);
    double comparisonAngle = atan2(comparison[1], comparison[0]);

    if (comparisonAngle > baseAngle)
        comparisonAngle -= 2 * PI;

    return baseAngle - comparisonAngle;
}


/* Returns a vector of all DLines in this m_pointMap */
// TODO: REWORK THIS LATER TO GET RID OF DOUBLED LINES
std::vector<DLine> DTriangulation::getLines() const
{
    std::vector<DLine> lineVector;
    for (std::map<int, DPoint>::const_iterator it = m_pointMap.begin();
         it != m_pointMap.end();
         ++it)
    {
        for (int idx: it->second.m_connections)
        {
            lineVector.push_back( DLine{it->second, m_pointMap.at(idx)} );
        }
    }
    return lineVector;
}


DTriangulation& DTriangulation::operator= (const DTriangulation &dtSource)
{
    m_pointMap = dtSource.m_pointMap;
    return *this;
}


void DTriangulation::createEdgeInMap(const int idx1,const int idx2)
{
    m_pointMap.at(idx1).createEdge(idx2);
    m_pointMap.at(idx2).createEdge(idx1);
}


void DTriangulation::deleteEdgeInMap(const int idx1,const int idx2)
{
    m_pointMap.at(idx1).deleteEdge(idx2);
    m_pointMap.at(idx2).deleteEdge(idx1);
}


/* Returns lines in the correct format for drawing/graphing the Delaunay Triangulation lines */
std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > DTriangulation::getLinesForDrawingOrGraphing()
{
    std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > lineVector;
    for ( auto line: getLines() )
    {
        lineVector.push_back( std::pair<Eigen::Vector3i, Eigen::Vector3i>
                              ( Eigen::Vector3i(line.m_xL, line.m_yL, 0),
                                Eigen::Vector3i(line.m_xR, line.m_yR, 0) ) );
    }
    return lineVector;
}


/* Uses the passed references to calculate the center and radius of a circle */
void calculateCircle(const DPoint &point, const DLine &line,
                     double &xCenter, double &yCenter, double &radius)
{
    // From here: http://paulbourke.net/geometry/circlesphere/
    //   variable names also taken from this paper
    double x1 = line.getLeftPoint().m_x;
    double y1 = line.getLeftPoint().m_y;
    double x2 = point.m_x;
    double y2 = point.m_y;
    double x3 = line.getRightPoint().m_x;
    double y3 = line.getRightPoint().m_y;
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
