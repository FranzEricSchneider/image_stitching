#include "delaunay_point.h"

//double DelaunayPoint::angleToConnection(const int idxOfPt)
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


void DelaunayPoint::deleteEdge(const int idxToDelete)
{
    std::vector<int>::iterator it;
    it = find(m_connections.begin(), m_connections.end(), idxToDelete);
    if ( it == m_connections.end() )
    {
        std::cout << "Tried to delete edge from " << m_idx << " to " << idxToDelete <<
                     ", but that edge doesn't exist\n";
    } else
    {
        std::cout << "Deleting edge from " << m_idx << " to " << idxToDelete << "\n";
        m_connections.erase(it);
    }
}


DelaunayPoint& DelaunayPoint::operator= (const DelaunayPoint &dpSource)
{
    if (this == &dpSource)
        return *this;

    m_idx = dpSource.m_idx;
    m_xy = dpSource.m_xy;
    m_connections = dpSource.m_connections;
    return *this;
}


std::ostream& operator<< (std::ostream &out, DelaunayPoint &dp)
{
    out << "DelaunayPoint [" << dp.m_idx << "], at " <<
           "(" << dp.m_xy.first << ", " << dp.m_xy.second   << "), connected to DelaunayPoints [";
    for (int i{}; i < static_cast<int>(dp.m_connections.size()); ++i)
    {
        std::cout << dp.m_connections[i];
        if (i != (static_cast<int>(dp.m_connections.size()) - 1) )
        {
            std::cout << ", ";
        }
    }
    std::cout << "]";
    return out;
}
