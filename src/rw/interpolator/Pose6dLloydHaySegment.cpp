/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "Pose6dLloydHaySegment.hpp"

#include <rw/math/Quaternion.hpp>

using namespace rw::interpolator;
using namespace rw::math;

Pose6dLloydHaySegment::Pose6dLloydHaySegment(const Pose6dStraightSegment& x1,
                                           const Pose6dStraightSegment& x2,
                                           double k,double tau)
                                           : _x1(x1),_x2(x2),_interval(0.0, 2*tau)
{
    _k = k;
    _tau = tau;
}


Pose6dLloydHaySegment::Pose6dLloydHaySegment(const Pose6dLloydHaySegment& segment):
    _x1(segment._x1),
    _x2(segment._x2),
    _interval(segment._interval),
    _k(segment._k),
    _tau(segment._tau)
{

}


Pose6dLloydHaySegment& Pose6dLloydHaySegment::operator=(const Pose6dLloydHaySegment& segment) {
    _x1 = segment._x1;
    _x2 = segment._x2;
    _k = segment._k;
    _tau = segment._tau;
    _interval = segment._interval;

    return *this;

}


Transform3D<> Pose6dLloydHaySegment::getX(double t) const {
    double s = t/(getIntervalLength());
    double s3 = s*s*s;
    double s4 = s3*s;
    double s5 = s4*s;
    double alfa = 6*s5-15*s4+10*s3;
    double s6 = s5*s;
    double beta = s6-3*s5+3*s4-s3;
    // Get position of blended path segments
    Transform3D<> x1 = _x1.getX(_x1.getIntervalLength()-_tau+t);
    Transform3D<> x2 = _x2.getX(t-_tau);

    Quaternion<> q1(x1.R());
    Quaternion<> q21 = (Quaternion<>(x2.R())- q1)*alfa;
    q21.normalize();
    Quaternion<> q = q1+q21;
    q.normalize();

    // Get velocity of the path segments
    VelocityScrew6D<> v1 = _x1.getXd(_x1.getIntervalLength()-_tau+t);
    VelocityScrew6D<> v2 = _x2.getXd(t-_tau);
    VelocityScrew6D<> vdkb = (v2-v1)*(_k*beta);

    Transform3D<> tvdkb(vdkb.linear(),vdkb.angular());

    Quaternion<> qres = q-Quaternion<>(tvdkb.R());
    qres.normalize();

    Vector3D<> pos( x1.P()+alfa*(x2.P()-x1.P())-tvdkb.P());
    return Transform3D<>(pos,qres.toRotation3D());
}

VelocityScrew6D<> Pose6dLloydHaySegment::getXd(double t) const {
    double s = t/(getIntervalLength());
    double s2 = s*s;
    double s3 = s2*s;
    double s4 = s3*s;
    double alfad = 30*s4-60*s3+30*s2;
    double s5 = s4*s;
    double betad = 6*s5-15*s4+12*s3-3*s2;

    double alfa = 6*s5-15*s4+10*s3;
    //    double s6 = s5*s;
    //    double beta = s6-3*s5+3*s4-s3;

    double index_x1 = _x1.getIntervalLength()-_tau+t;
    double index_x2 = t-_tau;

    // Get position of path segments
    Transform3D<> x1 = _x1.getX(index_x1);
    Transform3D<> x2 = _x2.getX(index_x2);

    Quaternion<> qx1(x1.R());
    Quaternion<> qx2(x2.R());

    Quaternion<> qxd( (qx2-qx1)*alfad );
    qxd.normalize();

    VelocityScrew6D<> vx21( Transform3D<>( (x2.P()-x1.P())*alfad , qxd.toRotation3D() ) );

    // Get velocity of path segments
    VelocityScrew6D<> v1 = _x1.getXd(index_x1);
    VelocityScrew6D<> v2 = _x2.getXd(index_x2);
    // No need to use acceleration on straight segments
    //Transform3D<> a1 = _x1.getXdd(index_x1);
    //Transform3D<> a2 = _x2.getXdd(index_x2);

    VelocityScrew6D<> vd = v2-v1;

    return vx21 - vd*_k*betad + vd*alfa + v1;

    //return alfad*(x2-x1)-_k*(v2-v1)*betad-_k*beta*(a2-a1)+v1+alfa*(v2-v1);
}

Transform3D<> Pose6dLloydHaySegment::getXdd(double t) const {
    return Transform3D<>::Identity();
}
