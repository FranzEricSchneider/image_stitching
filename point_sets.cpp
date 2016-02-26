#include "point_sets.h"


void PointSets::findMinimumsInBaseset()
{
    // Sets the minmax value (first) to the first element, and sets the
    //     corresponding index (second) to 0. Now each value will be compared
    //     to the first element
    m_minPointX.first = m_baseSet[0](0);
    m_minPointY.first = m_baseSet[0](1);
    m_minPointX.second = 0;
    m_minPointY.second = 0;

    for (int i{}; i < static_cast<int>(m_baseSet.size()); ++i)
    {
        if (m_baseSet[i](0) < m_minPointX.first)
        {
            m_minPointX.first = m_baseSet[i](0);
            m_minPointX.second = i;
        }
        if (m_baseSet[i](1) < m_minPointY.first)
        {
            m_minPointY.first = m_baseSet[i](1);
            m_minPointY.second = i;
        }
    }
}


void PointSets::generateConvexHullIndices(std::vector<int> &hullIndices)
{
    // Done using Graham scan: https://en.wikipedia.org/wiki/Graham_scan
    // Eigen examples: http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html

    int startingPointIndex = m_minPointY.second;
    Eigen::Vector3i startingPoint = m_baseSet[startingPointIndex];
    std::set< std::pair<double, int>, sortFirstElementDescending > anglesFromStartSet;
    for (int i{}; i < static_cast<int>(m_baseSet.size()); ++i)
    {
        if (i != startingPointIndex)
        {
            Eigen::Vector2d vec{ (m_baseSet[i] - startingPoint)[0],
                                 (m_baseSet[i] - startingPoint)[1] };
            vec.normalize();
            std::pair<double, int> xValue{vec[0], i};
            anglesFromStartSet.insert(xValue);
        }
    }

    // Initialize the convex hull with the starting point and the point closest to 0 deg from that
    std::set< std::pair<double, int> >::iterator setIt = anglesFromStartSet.begin();
    hullIndices = {startingPointIndex, (*setIt).second};  // Defined as a class element
    int checkIndex{};  // Check 0,1,2; then 1,2,3; etc.

    for (setIt = anglesFromStartSet.begin(); setIt != anglesFromStartSet.end(); /*nothing*/)
    {
        // Adds the next point to hullIndices
        if (setIt == --anglesFromStartSet.end())
        {
            // Close the loop by re-including the starting point
            hullIndices.push_back(startingPointIndex);
            ++setIt;
        } else
        {
            hullIndices.push_back( (*(++setIt)).second );
        }

        // If the added point has created a right turn, remove 2nd to last point from hullIndices
        while ( isRightTurn(m_baseSet[hullIndices[checkIndex]],
                            m_baseSet[hullIndices[checkIndex + 1]],
                            m_baseSet[hullIndices[checkIndex + 2]]) )
        {
            hullIndices.erase(hullIndices.begin() + checkIndex + 1);
            --checkIndex;
        }
        ++checkIndex;
    }
}


void PointSets::drawSet(std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > givenSet)
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
                     cv::Point2i(line.first(0), maxY - line.first(1)),
                     cv::Point2i(line.second(0), maxY - line.second(1)),
                     cv::Scalar(125, 125, 125, 0), 2);
        }
    }
}


void PointSets::graphSet(std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > givenSet)
{
    if (givenSet.empty())
    {
        std::cout << "The given set was empty, generate it first\n";
    }
    else
    {
        int numLines{static_cast<int>(givenSet.size())};
        m_plotTools.plotLines(numLines,
                              0, m_baseImg.size().width,
                              0, m_baseImg.size().height,
                              givenSet);
    }
}


void PointSets::drawBaseSet()
{
    int maxY = m_baseImg.size().height;
    for (auto point: m_baseSet)
    {
        cv::circle(m_baseImg, cv::Point2f(point(0), maxY - point(1)),
                   15, cv::Scalar(255, 255, 255, 0), 4);
    }
}


void PointSets::generateCompleteSet()
{
    for (int i{0}; i < static_cast<int>(m_baseSet.size()) - 1; ++i)
    {
        for (int j{i + 1}; j < static_cast<int>(m_baseSet.size()); ++j)
        {
            std::pair<Eigen::Vector3i, Eigen::Vector3i> line{m_baseSet[i], m_baseSet[j]};
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


void PointSets::generateConvexHull()
{
    std::vector<int> hullIndices;
    generateConvexHullIndices(hullIndices);
    int numLines{static_cast<int>(hullIndices.size() - 1)};  // One less line than points
    for (int i{}; i < numLines; ++i)
    {
        std::pair<Eigen::Vector3i, Eigen::Vector3i> line{m_baseSet[hullIndices[i]],
                                                         m_baseSet[hullIndices[i + 1]]};
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


void PointSets::generateDelaunay()
{
    m_dt = DTriangulation{m_baseSet};
}


void PointSets::drawDelaunay()
{
    drawSet(m_dt.getLinesForDrawingOrGraphing());
}


void PointSets::graphDelaunay()
{
    graphSet(m_dt.getLinesForDrawingOrGraphing());
}


// TODO: MAKE IT SO CIRCLES AREN'T DOUBLE PRINTED
void PointSets::drawDelaunayCircumcircles()
{
    std::map<int, DPoint>::iterator mapIt = m_dt.m_pointMap.begin();
    int maxY = m_baseImg.size().height;
    for (auto mapIt = m_dt.m_pointMap.begin(); mapIt!=m_dt.m_pointMap.end(); ++mapIt)
    {
        for (auto vecIt = mapIt->second.m_connections.begin();
             vecIt != mapIt->second.m_connections.end();
             /*Nothing*/)
        {
            double x, y, radius;
            int point1 = *vecIt;

            ++vecIt;
            if (vecIt == mapIt->second.m_connections.end())
                break;

            int point2 = *vecIt;
            if ( m_dt.m_pointMap[point1].isConnected(point2) )
            {
                DLine line{mapIt->second, m_dt.m_pointMap[point2]};
                calculateCircle(m_dt.m_pointMap[point1], line, x, y, radius);
                cv::circle(m_baseImg, cv::Point2f(x, maxY - y),
                           radius, cv::Scalar(255, 255, 255, 0), 2);
            }
        }
    }
}


void PointSets::showImage()
{
    cv::imshow(m_windowName, m_baseImg);
// TODO: MAKE IT ONLY DISAPPEAR ON ESC OR ENTER
    while (cv::waitKey(100) == -1) {}
}


// Takes the vector from a to b and from b to c, then checks whether a right turn occurred with cross
bool isRightTurn(Eigen::Vector3i a, Eigen::Vector3i b, Eigen::Vector3i c)
{
    Eigen::Vector3i ab = b - a;
    Eigen::Vector3i bc = c - b;
    Eigen::Vector3i abCrossbc = ab.cross(bc);
    return (abCrossbc[2] < 0.0);
}
