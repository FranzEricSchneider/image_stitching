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
    double lowestYVal{it->second.m_xy.second};

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


void DelaunayTriangulation::mergeGroups( DelaunayTriangulation leftSide,
                                         DelaunayTriangulation rightSide )
{
    copyConnectionsToThisMap(leftSide);
    copyConnectionsToThisMap(rightSide);
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
