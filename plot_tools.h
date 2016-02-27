#ifndef PLOT_TOOLS_H_INCLUDED
#define PLOT_TOOLS_H_INCLUDED

#include "gnuplot-iostream.h"
#include <Eigen/Geometry>


class PlotTools
{
    private:
        Gnuplot m_gp;

    public:
        void plotLines(const int numLines, const double xMin, const double xMax,
                       const double yMin, const double yMax,
                       const std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > &lines);
};

#endif // PLOT_TOOLS_H_INCLUDED
