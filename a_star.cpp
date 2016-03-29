#include "a_star.h"
#include "random_points.h"


void AStar::generateStartEndPoints()
{
    int numOuterPoints = static_cast<int>(m_convexHull.size());
    int halfOuterPoints = (numOuterPoints - 1) / 2;
    m_startIdx = randomNumber(halfOuterPoints);
    m_endIdx = m_startIdx + halfOuterPoints;
}


void AStar::generateStartingCostMap()
{
    if (m_startIdx == m_endIdx)
    {
        std::cout << "You need to generate start/end points first\n";
        exit(-1);
    }
    uint32_t maxVal{-1};
    for (std::map<int, DPoint>::iterator it = m_dt.m_pointMap.begin();
         it != m_dt.m_pointMap.end();
         ++it)
    {
        if (it->first == m_startIdx)
        {
            // The starting idx has a cost of zero
            m_distanceCostMap.insert( std::pair<int, int> (m_startIdx, 0) );
            m_totalCostMap.insert( std::pair<int, int> (m_startIdx, heuristicCost(m_startIdx)) );
        }
        else
        {
            // Every other idx starts off with a cost of infinity
            m_distanceCostMap.insert( std::pair<int, int> (it->first, static_cast<int>(maxVal)) );
            m_totalCostMap.insert( std::pair<int, int> (it->first, static_cast<int>(maxVal)) );
        }
    }
}


int AStar::heuristicCost(int locationIdx)
{
    // In order to ba a consistent heuristic, the heuristic must be less than the straight-line cost
    // https://en.wikipedia.org/wiki/Consistent_heuristic
    DPoint startPt = m_dt.m_pointMap.at(m_startIdx);
    DPoint endPt = m_dt.m_pointMap.at(m_endIdx);
    double dist = sqrt( pow(endPt.m_x - startPt.m_x, 2) + pow(endPt.m_y - startPt.m_y, 2) );
    if (dist > 1) --dist;
    return static_cast<int>(dist);
}


int AStar::getLowestCostOpenIdx()
{
    std::set<int>::const_iterator it = m_openSet.begin();
    int idx = *it;
    int lowestCost = m_totalCostMap.at(idx);
    for ( /*Already initialized*/;
          it != m_openSet.end();
          ++it )
    {
        if (*it < lowestCost)
        {
            idx = *it;
            lowestCost = m_totalCostMap.at(idx);
        }
    }
    return idx;
}


void AStar::generateFinalPath()
{
    // Write a thing that generates m_finalPath to m_endIdx
}


void AStar::moveIdxFromOpentoClosedSet(int idx)
{
    m_openSet.erase(idx);
    m_closedSet.insert(idx);
}


bool AStar::isIdxInSet(int idx, std::set<int> givenSet)
{
    std::set<int>::const_iterator it = givenSet.find(idx);
    return (it == givenSet.end());
}


AStar& AStar::operator= (const AStar &asSource)
{
    m_convexHull = asSource.m_convexHull;
    m_dt         = asSource.m_dt;
    m_startIdx   = asSource.m_startIdx;
    m_endIdx     = asSource.m_endIdx;
    m_finalPath  = asSource.m_finalPath;
    return *this;
}


void AStar::generateAStarPath()
{
    while (m_openSet.size() > 0)
    {
        int currentIdx = getLowestCostOpenIdx();
        DPoint currentPt = m_dt.m_pointMap.at(currentIdx);
        if (currentIdx == m_endIdx)
        {
            generateFinalPath();
            return;
        }
        moveIdxFromOpentoClosedSet(currentIdx);

        for (auto neighborIdx: currentPt.m_connections)
        {
            if ( isIdxInSet(neighborIdx) )
            {
                /* WORKING HERE */;
            }
        }
    }
}


int randomNumber(const int vMax, const int vMin)
{
    std::random_device m_rd;
    std::mt19937 m_mersenne{m_rd()};
    uint32_t maxVal{-1};
    static const double fraction = 1.0 / static_cast<double>(maxVal);
    return static_cast<int>(m_mersenne() * fraction * (vMax - vMin) + vMin);
}
