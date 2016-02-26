#include "april_analysis.h"
#include "point_sets.h"
#include "random_points.h"

// We want to find the
//    Shortest path between points
//    Voronoi diagram

int main()
{

/*
CURRENTLY REFACTORING IN POINT_SETS.CPP
TODO: MAKE A DELAUNAYTRIANGULATION FUNCTION THAT MAKES EDGES BETWEEN POINTS
MAKE DELAUNAY WORK
REFACTOR findFirstLine
CLEAN EVERYTHING AFTER DELAUNAY, MAKE NAMES AWESOME, COMMENT EVERYTHING
*/


//    AprilAnalysis april;
//    april.processImage();
//    PointSets ps{april.m_detections, april.m_img};
    RandomPoints random(9);
//    RandomPoints random;
    PointSets ps{random.m_randomDetections, random.m_img};

    ps.drawBaseSet();
//    ps.generateCompleteSet();
//    ps.drawCompleteSet();
//    ps.graphCompleteSet();
    ps.generateConvexHull();
//    ps.drawConvexHull();
    ps.graphConvexHull();
    ps.generateDelaunay();
    ps.drawDelaunay();
//    ps.graphDelaunay();
    ps.showImage();
    return 0;
}
