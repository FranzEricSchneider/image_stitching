#include "delauney_triangulation.h"

int pointWithLowestY()
{
    int idx{};
    double lowestYVal{m_pointSet[0]};

    for (int i{1}; i < static_cast<int>(m_pointSet.size()); ++i)
    {
        if (m_pointSet[i].m_xy.second < lowestYVal)
        {
            idx = i;
            lowestYVal = m_pointSet[i].m_xy.second
        }
    }

    return idx;
}
