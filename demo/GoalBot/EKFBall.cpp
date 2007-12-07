#include "EKFBall.hpp"

const double g = -9.82;
const double C_D = 0.45; //Drag coefficient
const double A = 0.002; //Front area of ball
const double M = 0.050; //Mass of ball
const double RHO = 1.29; //Air viscosity
const double ALPHA = 0*C_D*RHO*A/(2*M); //Coefficient for v^2 of acceleration due to air drag

using namespace boost::numeric::ublas;

EKFBall::EKFBall(): EKF(6) {

}

EKFBall::~EKFBall() {

}

Vector EKFBall::process(const Vector& x, double h) {
    _h = h;
    Vector xres(_n);
    for (size_t i = 0; i<3; i++)
	xres(i) += h*x(i+3);

    double vlength = sqrt(x(3)*x(3)+x(4)*x(4)+x(5)*x(5));
    xres(3) = x(3)-h*vlength*x(3)*ALPHA;
    xres(4) = x(4)-h*vlength*x(4)*ALPHA;
    xres(5) = x(5)+h*(g-vlength*x(5)*ALPHA);

    return xres;
}

Vector EKFBall::measurement(const Vector& x) {
    return x;
}

Matrix EKFBall::processJacobian(const Vector& x) {
    Matrix jac(_n, _n);
    
    jac(0,0) = 1;
    jac(0,1) = jac(0,2) = 0;
    jac(0,3) = _h;
    jac(0,4) = jac(0,5) = 0;

    jac(1,0) = 0;
    jac(1,1) = 1;
    jac(1,2) = jac(1,3) = 0;
    jac(1,4) = _h;
    jac(1,5) = 0;

    jac(2,0) = jac(2,1) = 0;
    jac(2,2) = 0;
    jac(2,3) = jac(2,4) = 0;
    jac(2,5) = _h;

    
    double vlength = sqrt(x(3)*x(3)+x(4)*x(4)+x(5)*x(5));

    jac(3,0) = jac(3,1) = jac(3,2) = 0;
    jac(3,3) = 1-_h*ALPHA*(vlength+2*x(3)*x(3)/vlength);
    jac(3,4) = -_h*ALPHA*2*x(4)/vlength;
    jac(3,5) = -_h*ALPHA*2*x(5)/vlength;

    jac(4,0) = jac(4,1) = jac(4,2) = 0;
    jac(4,3) = -_h*ALPHA*2*x(3)/vlength;
    jac(4,4) = 1-_h*ALPHA*(vlength+2*x(4)*x(4)/vlength);
    jac(4,5) = -_h*ALPHA*2*x(5)/vlength;

    jac(5,0) = jac(5,1) = jac(5,2) = 0;
    jac(5,3) = -_h*ALPHA*2*x(3)/vlength;
    jac(5,4) = -_h*ALPHA*2*x(4)/vlength;
    jac(5,5) = 1-_h*ALPHA*(vlength+2*x(5)*x(5)/vlength);    

    return jac;

}

Matrix EKFBall::processNoiseJacobian(const Vector& x) {
    return identity_matrix<double>(_n);
}

Matrix EKFBall::processNoiseCoVariance(const Vector& x) { //Q 
    return zero_matrix<double>(6,6);
}
    
Matrix EKFBall::measurementJacobian(const Vector& x) { //H
    return identity_matrix<double>(6);
}

Matrix EKFBall::measurementNoiseJacobian(const Vector& x) { //V
    return identity_matrix<double>(6);
}
