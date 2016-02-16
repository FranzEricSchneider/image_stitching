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

    /*FOR DEBUGGING*/
    std::map<int, DelaunayPoint>::iterator mapIt = m_pointMap.begin();
    for (mapIt = m_pointMap.begin(); mapIt!=m_pointMap.end(); ++mapIt)
    {
        std::cout << "Map point: " << mapIt->first << " => (" << mapIt->second.m_xy.first << ", " << mapIt->second.m_xy.second << ")\n";
        std::cout << "\tconnections: ";
        for (auto connection: mapIt->second.m_connections)
        {
            std::cout << connection << ", ";
        }
        std::cout << "\n";
    }
    std::cout << "Point with lowest Y (idx): " << pointWithLowestY() << "\n";
    std::cout << "\n";
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
        if (leftPoint == -1 && rightPoint == -1)
        {
            leftPoint  = leftSide .pointWithLowestY();
            rightPoint = rightSide.pointWithLowestY();
            // Look up copy constructor and make one for DelaunayLine
        }
        else
        {
        }
    }

    return leftLines[0];
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
            lineVector.push_back( DelaunayLine{it->first, idx, it->second, m_pointMap[idx]} );
        }
    }
    return lineVector;
}
