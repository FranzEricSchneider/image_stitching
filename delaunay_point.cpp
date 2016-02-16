#include "delaunay_point.h"

//double DelaunayPoint::angleToConnection(const int idxOfPt)
//{
//}
//
//void DelaunayPoint::deleteEdge(const int idxToDelete)
//{
//}

void DelaunayPoint::createEdge(const int idxOfPoint)
{
    std::vector<int>::iterator it;
    it = find(m_connections.begin(), m_connections.end(), idxOfPoint);
    if ( it == m_connections.end() )
    {
        m_connections.push_back(idxOfPoint);
    } else
    {
        std::cout << "Tried to create edge from " << m_idx << " to " << idxOfPoint <<
                     ", but that edge already existed\n";
    }
}


DelaunayPoint& DelaunayPoint::operator= (const DelaunayPoint &dpSource)
{
    // Do the copy
    m_idx = dpSource.m_idx;
    m_xy = dpSource.m_xy;
    m_connections = dpSource.m_connections;

    // Return the existing object
    return *this;
}
