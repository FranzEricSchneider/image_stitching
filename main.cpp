#include "april_analysis.h"
#include "point_sets.h"
#include "random_points.h"



#include "delaunay_line.h"



// We want to find the
//    Delaunay triangulation
//    Shortest path between points
//    Voronoi diagram

int main()
{

/*
PLUG THE FIRST LINE INTO THE TRIANGULATION AND KEEP GOING FROM THERE
MAKE DELAUNAY WORK
CHECK FOR VARIOUS TODOs
*/

//    AprilAnalysis april;
//    april.processImage();
//    PointSets ps{april.m_detections, april.m_img};
//    RandomPoints random(6);
    RandomPoints random;
    PointSets ps{random.m_randomDetections, random.m_img};




//std::map<int, DelaunayPoint> map1;
//map1.insert( std::pair<int, DelaunayPoint> (0, DelaunayPoint{1, 2}) );
//map1.insert( std::pair<int, DelaunayPoint> (1, DelaunayPoint{3, 4}) );
//map1.insert( std::pair<int, DelaunayPoint> (2, DelaunayPoint{5, 6}) );
//
//std::map<int, DelaunayPoint>::iterator it = map1.begin();
//std::map<int, DelaunayPoint> map2;
//map2.insert( std::pair<int, DelaunayPoint> (it->first, it->second) );
//
//std::map<int, DelaunayPoint> map3;
//map3 = map1;
//for (auto it3 = map3.begin(); it3 != map3.end(); ++it3)
//{
//    std::cout << "map3[" << it3->first << "]: (" << it3->second.m_xy.first << ", "<< it3->second.m_xy.second << ")\n";
//}
//
//
//std::cout << "map1[0].m_idx: " << map1[0].m_idx << "\n";
//std::cout << "map2[0].m_idx: " << map2[0].m_idx << "\n";
//map2[0].m_idx = 10;
//std::cout << "map1[0].m_idx: " << map1[0].m_idx << "\n";
//std::cout << "map2[0].m_idx: " << map2[0].m_idx << "\n";





    ps.drawBaseSet();
//    ps.generateCompleteSet();
//    ps.drawCompleteSet();
//    ps.graphCompleteSet();
//    ps.generateConvexHull();
//    ps.drawConvexHull();
//    ps.graphConvexHull();
    ps.showSetImage();
    return 0;
}
