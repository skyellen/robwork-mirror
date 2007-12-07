#ifndef EKFBALL_HPP
#define EKFBALL_HPP

#include "EKF.hpp"

class EKFBall: public EKF {
public:
    EKFBall();
    ~EKFBall();
private:
    double _h;

    virtual Vector process(const Vector& x, double dt); //f(x,u,w)

    virtual Vector measurement(const Vector& x); //h(x,v)

    virtual Matrix processJacobian(const Vector& x); //A
    
    virtual Matrix processNoiseJacobian(const Vector& x); //W

    virtual Matrix processNoiseCoVariance(const Vector& x); //Q    
    
    virtual Matrix measurementJacobian(const Vector& x); //H

    virtual Matrix measurementNoiseJacobian(const Vector& x); //V
};

#endif //#ifndef EKFBALL_HPP
