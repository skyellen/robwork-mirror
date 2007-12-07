#include "PA10.h"

namespace PA10 {
    /**
     * Calculates the base to tool jacobian of a PA10
     */ 
    /*  void jacobian(const boost::numeric::ublas::vector<double>& q, boost::numeric::ublas::matrix<double>& jac) {
	double s2 = sin(q(1));
	double s3 = sin(q(2));
	double s4 = sin(q(3));
	double s5 = sin(q(4));
	double s6 = sin(q(5));
	double s7 = sin(q(6));
	
	double c2 = cos(q(1));
	double c3 = cos(q(2));
	double c4 = cos(q(3));
	double c5 = cos(q(4));
	double c6 = cos(q(5));
	double c7 = cos(q(6));
	
	jac(0,0) = -(c2*c6*c7*d5*s4*s5) - c2*c7*d7*s4*s5 - c2*c7*d8*s4*s5 + c7*d3*s2*s3*s4*s6 + c4*d3*s2*s3*s5*s7 + d5*s2*s3*s5*s7 + c6*d7*s2*s3*s5*s7 + c6*d8*s2*s3*s5*s7 - c2*c4*d7*s6*s7 - c2*c4*d8*s6*s7 - c5*(c7*(c6*(c4*d3 + d5) + d7 + d8)*s2*s3 + (c3*(d3 + c4*(d5 + c6*(d7 + d8)))*s2 + c2*(d5 + c6*(d7 + d8))*s4)*s7) - c3*s2*(c6*c7*d3*s5 + c4*c7*(c6*d5 + d7 + d8)*s5 - (d7 + d8)*s4*s6*s7);
	
	jac(0,1) = c3*(-(c5*c7*(c6*(c4*d3 + d5) + d7 + d8)) + c7*d3*s4*s6 + (c4*d3 + d5 + c6*(d7 + d8))*s5*s7) + s3*(c6*c7*d3*s5 + (c5*d3 - (d7 + d8)*s4*s6)*s7 + c4*(c7*(c6*d5 + d7 + d8)*s5 + c5*(d5 + c6*(d7 + d8))*s7));

	jac(0,2) = -(c7*(c6*d5 + d7 + d8)*s4*s5) - (c5*(d5 + c6*(d7 + d8))*s4 + c4*(d7 + d8)*s6)*s7;

	jac(0,3) = -(c5*c7*(c6*d5 + d7 + d8)) + (d5 + c6*(d7 + d8))*s5*s7;
	jac(0,4) = -((d7 + d8)*s6*s7); 
	jac(0,5) = -(c7*(d7 + d8));
	jac(0,6) = 0;
	
	
	
	jac(1,0) = c2*(c4*(a7*c6 - c7*(d7 + d8)*s6) - s4*(c5*(c7*(d5 + c6*(d7 + d8)) + a7*s6) - (c6*d5 + d7 + d8)*s5*s7)) + s2*(s3*(c7*(c4*d3 + d5 + c6*(d7 + d8))*s5 + a7*s5*s6 + (c5*(c6*(c4*d3 + d5) + d7 + d8) - d3*s4*s6)*s7) + c3*(-(c5*(c7*(d3 + c4*(d5 + c6*(d7 + d8))) + a7*c4*s6)) + (d7 + d8)*(c7*s4*s6 + c4*s5*s7) + c6*(-(a7*s4) + (d3 + c4*d5)*s5*s7)));
	
	jac(1,1) = c3*c4*c7*d3*s5 + c3*c7*d5*s5 - c7*d7*s3*s4*s6 - c7*d8*s3*s4*s6 + a7*c3*s5*s6 - c4*d7*s3*s5*s7 - c4*d8*s3*s5*s7 - c3*d3*s4*s6*s7 + c5*(c7*(d3 + c4*(d5 + c6*(d7 + d8)))*s3 + c3*(c6*d5 + d7 + d8)*s7 + c4*(a7*s3*s6 + c3*c6*d3*s7)) + c6*(c3*c7*(d7 + d8)*s5 + s3*(a7*s4 - (d3 + c4*d5)*s5*s7));
	
	jac(1,2) = c4*(a7*c6 - c7*(d7 + d8)*s6) - s4*(c5*(c7*(d5 + c6*(d7 + d8)) + a7*s6) - (c6*d5 + d7 + d8)*s5*s7);
	
	jac(1,3) = c7*(d5 + c6*(d7 + d8))*s5 + a7*s5*s6 + c5*(c6*d5 + d7 + d8)*s7;
	jac(1,4) = a7*c6 - c7*(d7 + d8)*s6;
	jac(1,5) = (d7 + d8)*s7;
	jac(1,6) = a7;
	
	
	
	jac(2,0) = a7*c3*c4*c7*s2*s5 + a7*c2*c7*s4*s5 + c3*d3*s2*s5*s6 + c3*c4*d5*s2*s5*s6 + c2*d5*s4*s5*s6 + a7*c2*c4*s6*s7 - a7*c3*s2*s4*s6*s7 + c6*s2*s3*(d3*s4 - a7*s5*s7) + c5*(a7*c7*s2*s3 + a7*c2*c6*s4*s7 + s2*((c4*d3 + d5)*s3*s6 + a7*c3*c4*c6*s7));
	
	jac(2,1) = -(s3*(c4*(a7*c7*s5 + d5*s5*s6 + a7*c5*c6*s7) + s6*(d3*s5 - a7*s4*s7))) + c3*(c5*(a7*c7 + (c4*d3 + d5)*s6) + c6*(d3*s4 - a7*s5*s7));
	
	jac(2,2) = a7*c7*s4*s5 + a7*c4*s6*s7 + s4*(d5*s5*s6 + a7*c5*c6*s7);
	jac(2,3) = c5*(a7*c7 + d5*s6) - a7*c6*s5*s7;
	jac(2,4) = a7*s6*s7;
	jac(2,5) = a7*c7;
	jac(2,6) = 0;
	
	jac(3,0) = c7*(c6*(c5*(c3*c4*s2 + c2*s4) - s2*s3*s5) + (c2*c4 - c3*s2*s4)*s6) - (c5*s2*s3 + (c3*c4*s2 + c2*s4)*s5)*s7;
	jac(3,1) = c7*s3*s4*s6 - c3*(c6*c7*s5 + c5*s7) + c4*s3*(-(c5*c6*c7) + s5*s7);
	jac(3,2) = c5*c6*c7*s4 + c4*c7*s6 - s4*s5*s7;
	jac(3,3) = -(c6*c7*s5) - c5*s7;
	jac(3,4) = c7*s6;
	jac(3,5) = -s7;
	jac(3,6) = 0;
	
	jac(4,0) = -(c7*(c5*s2*s3 + (c3*c4*s2 + c2*s4)*s5)) - (c6*(c5*(c3*c4*s2 + c2*s4) - s2*s3*s5) + (c2*c4 - c3*s2*s4)*s6)*s7;
	jac(4,1) = c3*(-(c5*c7) + c6*s5*s7) + s3*(-(s4*s6*s7) + c4*(c7*s5 + c5*c6*s7));
	jac(4,2) = -(c7*s4*s5) - (c5*c6*s4 + c4*s6)*s7;
	jac(4,3) = -(c5*c7) + c6*s5*s7;
	jac(4,4) = -(s6*s7);
	jac(4,5) = -c7;
	jac(4,6) = 0;
	
	jac(5,0) = c2*(c4*c6 - c5*s4*s6) - s2*(-(s3*s5*s6) + c3*(c6*s4 + c4*c5*s6));
	jac(5,1) = c6*s3*s4 + (c4*c5*s3 + c3*s5)*s6;
	jac(5,2) = c4*c6 - c5*s4*s6;
	jac(5,3) = s5*s6;
	jac(5,4) = c6;
	jac(5,5) = 0;
	jac(5,6) = 1;
	
    } 
    */

}; //end namespace PA10
