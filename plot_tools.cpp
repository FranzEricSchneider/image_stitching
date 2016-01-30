#include "plot_tools.h"


void PlotTools::plotLines(const int numLines, const double xmin, const double xmax,
                          const double ymin, const double ymax,
                          const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > &lines)
{
    m_gp << "set xrange [" << xmin << ":" << xmax << "]\n";
    m_gp << "set yrange [" << ymin << ":" << ymax << "]\n";
    std::string plotString{"plot"};
    for (int i{}; i < numLines; ++i)
    {
//        plotString += " '-' with lines";
        plotString += " '-' with lines linetype rgb 'blue'";
        if (i != numLines - 1) { plotString += ","; }
    }
    plotString += " \n";
    m_gp << plotString;
    for (int i{}; i < numLines; ++i)
    {
        std::vector< std::pair<double, double> > line;
        line.push_back(std::make_pair(lines[i].first(0), lines[i].first(1)));
        line.push_back(std::make_pair(lines[i].second(0), lines[i].second(1)));
        m_gp.send1d(line);
    }
}
