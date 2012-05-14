/*
 * Pose2D.hpp
 *
 *  Created on: 18-08-2008
 *      Author: jimali
 */

#ifndef POSE2D_HPP_
#define POSE2D_HPP_

#include <rw/math/Transform2D.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <iostream>

class Pose2D
{
private:
    rw::math::Vector2D<> _pos;
    double _theta;

public:
    Pose2D() :
        _pos(0,0),
        _theta(0)
    {}

    Pose2D(rw::math::Vector2D<> pos, double theta) :
        _pos(pos),
        _theta(theta)
    {}

    Pose2D(double x, double y, double theta) :
        _pos(x,y), _theta(theta)
    {}

    Pose2D(const rw::math::Transform2D<>& transform) :
        _pos(transform.P()),
        _theta(
            // Sigh.
            atan2(
                transform.R()(1, 0),
                transform.R()(0, 0)))
    {}

    double& x(){return _pos[0];}
    double& y(){return _pos[1];}
    double& theta(){return _theta;}
    rw::math::Vector2D<>& getPos(){return _pos;}

    double x() const { return _pos[0]; }
    double y() const { return _pos[1]; }
    double theta() const { return _theta; }
    const rw::math::Vector2D<>& getPos() const { return _pos; }

    // The transform corresponding to the pose.
    static
    rw::math::Transform2D<> transform(const Pose2D& pose);

    static
    void print(const Pose2D& pose);

    // A vector of (x, y, theta).
    static
    boost::numeric::ublas::vector<double> toUblas(const Pose2D& pose);
};

#endif /* POSE2D_HPP_ */
