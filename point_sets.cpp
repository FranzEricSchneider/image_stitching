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
    m_minPointX.first = m_maxPointX.first = m_baseSet[0].x;
    m_minPointY.first = m_maxPointY.first = m_baseSet[0].y;
    m_minPointX.second = m_maxPointX.second = 0;
    m_minPointY.second = m_maxPointY.second = 0;

    for (int i{}; i < static_cast<int>(m_baseSet.size()); ++i)
    {
        if (m_baseSet[i].x < m_minPointX.first)
        {
            m_minPointX.first = m_baseSet[i].x;
            m_minPointX.second = i;
        }
        else if (m_baseSet[i].x > m_maxPointX.first)
        {
            m_maxPointX.first = m_baseSet[i].x;
            m_maxPointX.second = i;
        }
        if (m_baseSet[i].y < m_minPointY.first)
        {
            m_minPointY.first = m_baseSet[i].y;
            m_minPointY.second = i;
        }
        else if (m_baseSet[i].y > m_maxPointY.first)
        {
            m_maxPointY.first = m_baseSet[i].y;
            m_maxPointY.second = i;
        }
    }
}


void PointSets::drawBaseSet()
{
    for (auto point: m_baseSet)
    {
        cv::circle(m_baseImg, cv::Point2f(point.x, point.y),
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
            std::pair<Point2D, Point2D> line{m_baseSet[i], m_baseSet[j]};
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
                     cv::Point2f(line.first.x, line.first.y),
                     cv::Point2f(line.second.x, line.second.y),
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
        m_gp << "set xrange [" << (m_minPointX.first - 20) << ":" << (m_maxPointX.first + 20) << "]\n";
        m_gp << "set yrange [" << (m_minPointY.first - 20) << ":" << (m_maxPointY.first + 20) << "]\n";
        std::string plotString{"plot"};
        for (int i{}; i < numLines; ++i)
        {
            plotString += " '-' with lines";
            if (i != numLines - 1) { plotString += ","; }
        }
        plotString += " \n";
        m_gp << plotString;
        for (int i{}; i < numLines; ++i)
        {
            std::vector< std::pair<double, double> > line;
            line.push_back(std::make_pair(m_completeSet[i].first.x, m_completeSet[i].first.y));
            line.push_back(std::make_pair(m_completeSet[i].second.x, m_completeSet[i].second.y));
            m_gp.send1d(line);
        }
    }
}


void PointSets::showSetImage()
{
    cv::imshow(m_windowName, m_baseImg);
    while (cv::waitKey(100) == -1) {}
}
