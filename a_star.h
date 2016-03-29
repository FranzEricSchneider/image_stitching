#ifndef A_STAR_H_INCLUDED
#define A_STAR_H_INCLUDED


#include <random>
#include <Eigen/Geometry>

#include "d_triangulation.h"
#include "d_point.h"


class AStar
{
    private:
        std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > m_convexHull{};
        DTriangulation m_dt{};
        std::map<int, int> m_distanceCostMap;  // Stores the distance cost to each node as <idx, score>. Idx matches m_dt
        std::map<int, int> m_totalCostMap;  // Stores the distance + heuristic cost to each node as <idx, score>
//        std::map<int, int> cameFromMap;  // I don't know yet what is supposed to go here
        int m_startIdx{}, m_endIdx{};
        std::set<int> m_closedSet;
        std::set<int> m_openSet;

        void generateStartEndPoints();
        void generateStartingCostMap();
        int heuristicCost(int start);
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

            m_finalPath.push_back( std::pair<Eigen::Vector3i, Eigen::Vector3i>
                                 ( m_convexHull.at(m_startIdx).first,
                                   m_convexHull.at(m_endIdx).first ) );
        }

        void generateAStarPath();
};


int randomNumber(const int vMax, const int vMin = 0);


#endif // A_STAR_H_INCLUDED
