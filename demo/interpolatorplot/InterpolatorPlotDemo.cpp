#include <rw/interpolator/CubicSplineInterpolator.hpp>
#include <rw/interpolator/StraightInterpolator.hpp>

#include <boost/numeric/ublas/io.hpp>

#include <fstream>

using namespace rw::interpolator;

int main(){

    // write GNU-plot datafile
    std::ofstream cubic("cubic.dat");
    std::ofstream line("line.dat");

    Interpolator::Point p[10];
    for(size_t i = 0; i<10; i++){
        p[i] = Interpolator::Point(1);
    }
    p[0][0] = 13.0;
    p[1][0] = 4.0;
    p[2][0] = 8.0;
    p[3][0] = 11.0;
    p[4][0] = 19.0;
    p[5][0] = 14.0;
    p[6][0] = 1.0;
    p[7][0] = 23.0;
    p[8][0] = 2.0;
    p[9][0] = 3000.0;
    
    Interpolator::ViaPointList list;
    for(size_t i = 0; i<10; i++){
        list.push_back(p[i]);
    }

    CubicSplineInterpolator cubicInterpolator(list);
    StraightInterpolator lineInterpolator(list);


    std::cout << "cubic length: " << cubicInterpolator.getLength() << "line lenght: " << lineInterpolator.getLength() << std::endl;

    cubic << "# Cubicspline plot" << std::endl;
    line << "# Line plot" << std::endl;

    double i;
    InterpolatorIterator it;

    double stepSize = 0.1;

    i = 0;
    it = cubicInterpolator.getIterator();
    while(!it.isEnd()){
        cubic << i << "\t" << it.getX()[0] << "\t" << it.getXd()[0] << "\t" << it.getXdd()[0] << std::endl;
        it += stepSize;
        i += stepSize;
    }

    i = 0;
    it = lineInterpolator.getIterator();
    while(!it.isEnd()){
        line << i << "\t" << it.getX()[0] << "\t" << it.getXd()[0] << "\t" << it.getXdd()[0] << std::endl;
        it += stepSize;
        i += stepSize;
    }


    return 0;
}
