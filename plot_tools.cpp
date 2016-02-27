#include "plot_tools.h"


void PlotTools::plotLines(const int numLines, const double xMin, const double xMax,
                          const double yMin, const double yMax,
                          const std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > &lines)
{
    m_gp << "set xrange [" << xMin << ":" << xMax << "]\n";
    m_gp << "set yrange [" << yMin << ":" << yMax << "]\n";
    std::string plotString{"plot"};
    for (int i{}; i < numLines; ++i)
    {
        plotString += " '-' with lines linetype rgb 'blue'";
        if (i != numLines - 1) { plotString += ","; }
    }
    plotString += " \n";
    m_gp << plotString;
    for (int i{}; i < numLines; ++i)
    {
        std::vector< std::pair<int, int> > line;
        line.push_back(std::make_pair( lines[i].first(0),  lines[i].first(1)  ));
        line.push_back(std::make_pair( lines[i].second(0), lines[i].second(1) ));
        m_gp.send1d(line);
    }
}
