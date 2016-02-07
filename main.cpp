#include "april_analysis.h"
#include "point_sets.h"
#include "random_points.h"


// We want to find the
//    Delaunay triangulation
//    Shortest path between points
//    Voronoi diagram

int main()
{

/*
LOOK UP HOW TO DO OVERLOADED CONSTRUCTORS so you don't need a placeholder
DEBUG WHY SOME RIGHT TURNS ARE GETTING INTO THE CONVEX HULL
*/

//    AprilAnalysis april;
//    april.processImage();
//    PointSets ps{april.m_detections, april.m_img};
//    RandomPoints random(20);
    RandomPoints random("placeholder");
    PointSets ps{random.m_randomDetections, random.m_img};

    ps.generateCompleteSet();
    ps.drawCompleteSet();
//    ps.graphCompleteSet();
    ps.generateConvexHull();
//    ps.drawConvexHull();
    ps.graphConvexHull();
    ps.showSetImage();
    return 0;
}
