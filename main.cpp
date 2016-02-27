#include "april_analysis.h"
#include "point_sets.h"
#include "random_points.h"

// We want to find the
//    Shortest path between points
//    Voronoi diagram

int main()
{

/*
CURRENTLY REFACTORING
    (point_sets, random_points, simple_stitcher, plot_tools, pair_comparison,
     d_point, )
TODO: MAKE A DELAUNAYTRIANGULATION FUNCTION THAT MAKES EDGES BETWEEN POINTS
MAKE DELAUNAY WORK
REFACTOR findFirstLine
CLEAN EVERYTHING AFTER DELAUNAY, MAKE NAMES AWESOME, COMMENT EVERYTHING
*/


//    AprilAnalysis april;
//    april.processImage();
//    PointSets ps{april.m_detections, april.m_img};
    RandomPoints random(4);
//    RandomPoints random;
    PointSets ps{random.m_randomDetections, random.m_img};

    ps.generateCompleteSet();
    ps.drawCompleteSet();
//    ps.graphCompleteSet();
    ps.generateDelaunay();
    ps.drawDelaunayCircumcircles();
    ps.drawDelaunay();
//    ps.graphDelaunay();
    ps.generateConvexHull();
    ps.drawConvexHull();
//    ps.graphConvexHull();
    ps.drawBaseSet();
    ps.showImage();
    return 0;
}
