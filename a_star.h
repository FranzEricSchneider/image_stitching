#ifndef A_STAR_H_INCLUDED
#define A_STAR_H_INCLUDED


#include <random>
#include <Eigen/Geometry>

#include "d_triangulation.h"
#include "d_point.h"


// https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
class AStar
{
    private:
        std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > m_convexHull{};
        DTriangulation m_dt{};
        std::map<int, double> m_distCostMap;  // Stores the dist cost to each node as <idx, score>. Idx matches m_dt
        std::map<int, double> m_totalCostMap;  // Stores the dist + heuristic cost to each node as <idx, score>
        std::map<int, int> m_cameFromMap;  // Stores most efficient last step to idx1 (key), which is idx2 (val)
        int m_startIdx{}, m_endIdx{};
        std::set<int> m_closedSet;
        std::set<int> m_openSet;

        void generateStartEndPoints();
        void generateStartingCostMap();
        double heuristicCost(int start);
        double distBetweenIdxPts(int idx1, int idx2);
        int getLowestCostOpenIdx();
        void generateFinalPath();
        void moveIdxFromOpentoClosedSet(int idx);
        bool isIdxInSet(int idx, std::set<int> givenSet);

    public:
        std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > m_finalPath{};

        AStar& operator= (const AStar &asSource);

        AStar() { /*Empty constructor*/ }

        AStar(std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > convexHull,
              DTriangulation dt):
        m_convexHull{convexHull}, m_dt{dt}
        {
            generateStartEndPoints();
            generateStartingCostMap();
            m_openSet.insert(m_startIdx);  // Start investigating the start idx
            generateAStarPath();
        }

        void generateAStarPath();
};


int randomNumber(const int vMax, const int vMin = 0);


#endif // A_STAR_H_INCLUDED
