#include "april_analysis.h"
#include "point_sets.h"


// We want to find the
//    Convex hull
//    Delaunay triangulation
//    Voronoi diagram

int main()
{
    AprilAnalysis april;
    april.processImage();
    PointSets ps{april.m_detections, april.m_img};
//    ps.generateCompleteSet();
//    ps.drawCompleteSet();
//    ps.graphCompleteSet();
//    ps.showSetImage();
    ps.generateConvexHullIndices();

//    std::vector<int> a{1, 2, 3, 4, 5, 6, 7, 8};
//    for (auto integer: a)
//    {
//        std::cout << integer;
//    }
//    std::cout << " ";
//    for (auto integer: a)
//    {
//        std::cout << integer;
//    }
//    std::cout << "\n";
//
//    std::vector<int>::iterator it;
//    for (it = a.begin(); it != a.end(); ++it)
//    {
//        if (it == --a.end())
//            std::cout << "Hey!";
//        std::cout << *it;
//        std::cout << *(--a.end());
//        std::cout << *(a.end() - 1);
//    }
    return 0;
}
