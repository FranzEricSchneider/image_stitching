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
    ps.generateConvexHull();
    return 0;
}
