#ifndef PLOT_TOOLS_H_INCLUDED
#define PLOT_TOOLS_H_INCLUDED

#include "gnuplot-iostream.h"
#include <Eigen/Geometry>


class PlotTools
{
    private:
        Gnuplot m_gp;

    public:
        void plotLines(const int numLines, const double xmin, const double xmax,
                       const double ymin, const double ymax,
                       const std::vector< std::pair<Eigen::Vector3i, Eigen::Vector3i> > &lines);
};

#endif // PLOT_TOOLS_H_INCLUDED
