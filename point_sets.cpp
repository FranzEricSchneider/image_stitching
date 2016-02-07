#include "point_sets.h"


void PointSets::print()
{
    std::cout << "The base set:\n";
    for (auto point: m_baseSet)
    {
        std::cout << "point:\n" << point << "\n";
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


void PointSets::drawSet(std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > givenSet)
{
    if (givenSet.empty())
    {
        std::cout << "Your given set was empty, generate it first\n";
    }
    else
    {
        int maxY = m_baseImg.size().height;
        for (auto line: givenSet)
        {
            cv::line(m_baseImg,
                     cv::Point2f(line.first(0), maxY - line.first(1)),
                     cv::Point2f(line.second(0), maxY - line.second(1)),
                     cv::Scalar(255, 0, 0, 0), 2 );
        }
    }
}


void PointSets::graphSet(std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > givenSet)
{
    if (givenSet.empty())
    {
        std::cout << "The given set was empty, generate it first\n";
    }
    else
    {
        int numLines{static_cast<int>(givenSet.size())};
        m_plotTools.plotLines(numLines,
                              m_minPointX.first - 100, m_maxPointX.first + 100,
                              m_minPointY.first - 100, m_maxPointY.first + 100,
                              givenSet);
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
    drawSet(m_completeSet);
}


void PointSets::graphCompleteSet()
{
    graphSet(m_completeSet);
}


void PointSets::generateConvexHullIndices()
{
    // Done using Graham scan: https://en.wikipedia.org/wiki/Graham_scan
    // Eigen examples: http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html

    int numPoints = m_baseSet.size();
    int startingPointIndex = m_minPointY.second;
    Eigen::Vector3d startingPoint = m_baseSet[startingPointIndex];
    std::set< std::pair<double, int>, pairComparison > anglesFromStartSet;
    for (int i{}; i < numPoints; ++i)
    {
        if (i != startingPointIndex)
        {
            Eigen::Vector3d vec = m_baseSet[i] - startingPoint;
            vec.normalize();
            std::pair<double, int> xValue{vec(0), i};
            anglesFromStartSet.insert(xValue);
        }
    }

    // Initialize the convex hull with the starting point and the point closest to 0 deg from that
    std::set< std::pair<double, int> >::iterator setIterator = anglesFromStartSet.begin();
    m_hullIndices = {startingPointIndex, (*setIterator).second};  // Defined as a class element
    int checkIndex{};  // Check 0,1,2; then 1,2,3; etc.

    for (setIterator = anglesFromStartSet.begin(); setIterator != anglesFromStartSet.end(); /*nothing*/)
    {
        // Adds the next point to m_hullIndices
        if (setIterator == --anglesFromStartSet.end())
        {
            // Close the loop by re-including the starting point
            m_hullIndices.push_back(startingPointIndex);
            ++setIterator;
        } else
        {
            m_hullIndices.push_back( (*(++setIterator)).second );
        }

        // If the added point has created a right turn, remove 2nd to last point from m_hullIndices
        while ( isRightTurn(m_baseSet[m_hullIndices[checkIndex]],
                            m_baseSet[m_hullIndices[checkIndex + 1]],
                            m_baseSet[m_hullIndices[checkIndex + 2]]) )
        {
            m_hullIndices.erase(m_hullIndices.begin() + checkIndex + 1);
            --checkIndex;
        }
        ++checkIndex;
    }
}


void PointSets::generateConvexHull()
{
    generateConvexHullIndices();
    int numLines{static_cast<int>(m_hullIndices.size() - 1)};  // One less line than points
    for (int i{}; i < numLines; ++i)
    {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> line{m_baseSet[m_hullIndices[i]],
                                                         m_baseSet[m_hullIndices[i + 1]]};
        m_convexHull.push_back(line);
    }
}


void PointSets::drawConvexHull()
{
    drawSet(m_convexHull);
}


void PointSets::graphConvexHull()
{
    graphSet(m_convexHull);
}


void PointSets::showSetImage()
{
    cv::imshow(m_windowName, m_baseImg);
    while (cv::waitKey(100) == -1) {}
}


// Takes the vector from a to b and from b to c, then checks whether a right turn occurred with cross
bool isRightTurn(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
{
    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d bc = c - b;
    Eigen::Vector3d abCrossbc = ab.cross(bc);
    return (abCrossbc[2] < 0.0);
}
