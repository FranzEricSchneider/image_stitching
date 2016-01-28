#include "april_analysis.h"
#include "simple_stitcher.h"


int main()
{
    SimpleStitcher ss;
    ss.stitchImagesAndOutput();

//    cv::Mat pano = cv::imread("imgs/a7.jpg"); // a2, a5
//    AprilAnalysis april;
//    april.setParameters(pano.rows, pano.cols);
//    april.setup();
//    april.processAndShowImage(pano);

    return 0;
}
