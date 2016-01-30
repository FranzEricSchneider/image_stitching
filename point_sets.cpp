#include "point_sets.h"


void PointSets::print()
{
    std::cout << "The base set:\n";
    for (auto point: m_baseSet)
    {
        std::cout << point << "\n";
    }
}


void PointSets::generateMinMax()
{
    // Sets the minmax value (first) to the first element, and sets the
    //     corresponding index (second) to 0. Now each value will be compared
    //     to the first element
    m_minPointX.first = m_maxPointX.first = m_baseSet[0](0);
    m_minPointY.first = m_maxPointY.first = m_baseSet[0](1);
    m_minPointX.second = m_maxPointX.second = 0;
    m_minPointY.second = m_maxPointY.second = 0;

    for (int i{}; i < static_cast<int>(m_baseSet.size()); ++i)
    {
        if (m_baseSet[i](0) < m_minPointX.first)
        {
            m_minPointX.first = m_baseSet[i](0);
            m_minPointX.second = i;
        }
        else if (m_baseSet[i](0) > m_maxPointX.first)
        {
            m_maxPointX.first = m_baseSet[i](0);
            m_maxPointX.second = i;
        }
        if (m_baseSet[i](1) < m_minPointY.first)
        {
            m_minPointY.first = m_baseSet[i](1);
            m_minPointY.second = i;
        }
        else if (m_baseSet[i](1) > m_maxPointY.first)
        {
            m_maxPointY.first = m_baseSet[i](1);
            m_maxPointY.second = i;
        }
    }
}


void PointSets::drawBaseSet()
{
    for (auto point: m_baseSet)
    {
        cv::circle(m_baseImg, cv::Point2f(point(0), point(1)),
                   15, cv::Scalar(0, 0, 255, 0), 4);
    }
}


void PointSets::generateCompleteSet()
{
    int numPts = m_baseSet.size();
    for (int i{0}; i < numPts - 1; ++i)
    {
        for (int j{i + 1}; j < numPts; ++j)
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> line{m_baseSet[i], m_baseSet[j]};
            m_completeSet.push_back(line);
        }
    }
}


void PointSets::drawCompleteSet()
{
    drawBaseSet();
    if (m_completeSet.empty())
    {
        std::cout << "Your complete set was empty, generate it first\n";
    }
    else
    {
        for (auto line: m_completeSet)
        {
            cv::line(m_baseImg,
                     cv::Point2f(line.first(0), line.first(1)),
                     cv::Point2f(line.second(0), line.second(1)),
                     cv::Scalar(255, 0, 0, 0), 2 );
        }
    }
}


void PointSets::graphCompleteSet()
{
    if (m_completeSet.empty())
    {
        std::cout << "Your complete set was empty, generate it first\n";
    }
    else
    {
        int numLines{static_cast<int>(m_completeSet.size())};
        m_plotTools.plotLines(numLines,
                              m_minPointX.first - 100, m_maxPointX.first + 100,
                              m_minPointY.first - 100, m_maxPointY.first + 100,
                              m_completeSet);
    }
}


void PointSets::generateConvexHull()
{
    // Done using Graham scan: https://en.wikipedia.org/wiki/Graham_scan
    // Eigen examples: http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html

    int numPoints = m_baseSet.size();
    int startingPointIndex = m_minPointY.second;
    Eigen::Vector3d startingPoint = m_baseSet[startingPointIndex];
    std::set< std::pair<double, int>, pairComparison > anglesFromPSet; // Uses pairComparison to sort pairs
    for (int i{}; i < numPoints; ++i)
    {
        if (i != startingPointIndex)
        {
            Eigen::Vector3d vec = m_baseSet[i] - startingPoint;
            vec.normalize();
            std::pair<double, int> xValue{vec(0), i};  // Sorts based on x value
            anglesFromPSet.insert(xValue);
        }
    }

//    std::set< std::pair<double, int> >::iterator it;
//    std::cout << "myset contains:";
//    for (it=anglesFromPSet.begin(); it!=anglesFromPSet.end(); ++it)
//        std::cout << ' ' << (*it).first;
//    std::cout << '\n';
}


void PointSets::showSetImage()
{
    cv::imshow(m_windowName, m_baseImg);
    while (cv::waitKey(100) == -1) {}
}
