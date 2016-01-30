#include "april_analysis.h"
#include "point_sets.h"


int main()
{

    AprilAnalysis april;
    april.processImage();
    PointSets ps{april.m_detections, april.m_img};
//    ps.print();
    ps.generateCompleteSet();
//    ps.drawCompleteSet();
    ps.graphCompleteSet();
//    ps.showSetImage();

    return 0;
}
