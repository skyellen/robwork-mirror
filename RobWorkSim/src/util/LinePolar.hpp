#ifndef LN_LINE_POLAR_HPP
#define LN_LINE_POLAR_HPP

#include "Pose2D.hpp"
#include "P2D.hpp"
#include <boost/numeric/ublas/vector.hpp>

class LinePolar
{
public:
    // rho * (cos(theta), sin(theta)) is the point on the line nearest to origo.
    LinePolar(double rho = 0, double theta = 0);

    // 'pnt' is any point on the line, and theta is the usual angle.
    static LinePolar make(const P2D& pnt, double theta);

    // The line moving through the segment from 'start' to 'end'.
    static LinePolar make(const P2D& start, const P2D& end);

    double getRho() const { return _rho; }
    double getTheta() const { return _theta; }
    const P2D& getNormal() const { return _normal; }

    // The L_2 distance from 'pnt' to the line.
    double dist2(const P2D& pnt);

    // The point for the projection of 'pnt' onto 'line'.
    static
    P2D projectionPoint(const LinePolar& line, const P2D& pnt);

    // A supporting point on the line (equal to rho * normal).
    static
    P2D linePoint(const LinePolar& line);

    // The vector for the projection of 'pnt' onto the normal of 'line'.
    static
    P2D normalProjectionVector(const LinePolar& line, const P2D& pnt);

    // Print the line to stdout.
    static
    void print(const LinePolar& line);

    // An ublas vector of (rho, theta).
    static
    boost::numeric::ublas::vector<double> toUblas(const LinePolar& line);

    // 'line' given relative to the coordinate frame of 'pose'.
    static
    LinePolar lineToLocal(
        const Pose2D& pose,
        const LinePolar& line);

public:
    double _rho;
    double _theta;
    P2D _normal;
};

#endif
