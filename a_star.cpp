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
            m_distCostMap.insert( std::pair<int, double> (m_startIdx, 0.0) );
            m_totalCostMap.insert( std::pair<int, double> (m_startIdx, heuristicCost(m_startIdx)) );
        }
        else
        {
            // Every other idx starts off with a cost of infinity
            m_distCostMap.insert( std::pair<int, double> (it->first, static_cast<double>(maxVal)) );
            m_totalCostMap.insert( std::pair<int, double> (it->first, static_cast<double>(maxVal)) );
        }
    }
}


double AStar::heuristicCost(int locationIdx)
{
    // In order to ba a consistent heuristic, the heuristic must be less than the straight-line cost
    // https://en.wikipedia.org/wiki/Consistent_heuristic

    DPoint startPt = m_dt.m_pointMap.at(m_startIdx);
    DPoint endPt = m_dt.m_pointMap.at(m_endIdx);
    double dist = distBetweenIdxPts(m_startIdx, m_endIdx);
    if (dist > 1) --dist;
    return dist;
}


double AStar::distBetweenIdxPts(int idx1, int idx2)
{
    DPoint pt1 = m_dt.m_pointMap.at(idx1);
    DPoint pt2 = m_dt.m_pointMap.at(idx2);
    return sqrt( pow(pt2.m_x - pt1.m_x, 2) + pow(pt2.m_y - pt1.m_y, 2) );;
}


int AStar::getLowestCostOpenIdx()
{
    std::set<int>::const_iterator it = m_openSet.begin();
    int idx = *it;
    double lowestCost = m_totalCostMap.at(idx);
    for ( /*Already initialized*/;
          it != m_openSet.end();
          ++it )
    {
        if (m_totalCostMap.at(*it) < lowestCost)
        {
            idx = *it;
            lowestCost = m_totalCostMap.at(idx);
        }
    }
    return idx;
}


void AStar::generateFinalPath()
{
    std::cout << "Made it to generateFinalPath!\n";

    // Wipes path if non-zero
    if (static_cast<int>(m_finalPath.size()) > 0)
    {
        std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > newPath{};
        m_finalPath = newPath;
    }

    int currentIdx = m_endIdx;
    while (currentIdx != m_startIdx)
    {
        int previousIdx = m_cameFromMap.at(currentIdx);
        DPoint dest{ m_dt.m_pointMap.at(currentIdx) };
        Eigen::Vector3i destEigen{dest.m_x, dest.m_y, 0};
        DPoint src{ m_dt.m_pointMap.at(previousIdx) };
        Eigen::Vector3i srcEigen{src.m_x, src.m_y, 0};
        m_finalPath.push_back( std::pair<Eigen::Vector3i, Eigen::Vector3i>(destEigen, srcEigen) );

        currentIdx = previousIdx;
    }
}


void AStar::moveIdxFromOpentoClosedSet(int idx)
{
    m_openSet.erase(idx);
    m_closedSet.insert(idx);
}


bool AStar::isIdxInSet(int idx, std::set<int> givenSet)
{
    auto it = givenSet.find(idx);
    return (it != givenSet.end());
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


/* WORKING HERE -- AStar is finding an empty path, use the debug tools! */
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
            if ( isIdxInSet(neighborIdx, m_closedSet) )
                continue;  // If the neighbor is in the closed set it has already been processed

            int tentativeDistScore = m_distCostMap.at(currentIdx) +
                                     distBetweenIdxPts(currentIdx, neighborIdx);

            if ( !isIdxInSet(neighborIdx, m_openSet) )
                m_openSet.insert(neighborIdx);
            else if ( tentativeDistScore >= m_distCostMap.at(neighborIdx) )
                continue; // tentative path is not better than current path

            // At this point the neighbor is in the open set and we know the best path to it
            m_cameFromMap[neighborIdx]  = currentIdx;  // This syntax overrides current value or adds new key
            m_distCostMap[neighborIdx]  = tentativeDistScore;
            m_totalCostMap[neighborIdx] = tentativeDistScore + heuristicCost(neighborIdx);
        }
    }
    std::cout << "AStar failed to find a path!\n";
}


int randomNumber(const int vMax, const int vMin)
{
    std::random_device m_rd;
    std::mt19937 m_mersenne{m_rd()};
    uint32_t maxVal{-1};
    static const double fraction = 1.0 / static_cast<double>(maxVal);
    return static_cast<int>(m_mersenne() * fraction * (vMax - vMin) + vMin);
}
