#include "VelRampProfile.hpp"

#include <cmath>

VelRampProfile::VelRampProfile(const rw::math::Vector2D<>& poslimits, const rw::math::Vector2D<>& vellimits, const rw::math::Vector2D<>& acclimits) :
    _poslimits(poslimits),
    _vellimits(vellimits),
    _acclimits(acclimits) 
{
}

VelRampProfile::~VelRampProfile() {

}




    


double VelRampProfile::getVelocity(double goal, double q, double dq, double h) const {
    double x =  std::min(std::max(goal, _poslimits(0)), _poslimits(1))-q;
    if (x>0) { //Needs a positive velocity
	
	//For qmax
	double j_x = round(sqrt(1-8*x/(h*h*_acclimits(0)))/2-1);
	double q_end_x = (x+h*h*_acclimits(0)*(j_x*(j_x+1))/2)/(h*(j_x+1));
	double q_max_x = q_end_x-j_x*_acclimits(0)*h;
	double X = x-h*q_max_x;
	double posmax = 0;
	if (X>0) {
	    double j_X = round(sqrt(1.-8*X/(h*h*_acclimits(0)))/2.-1);
	    double q_end_X = (X+h*h*_acclimits(0)*(j_X*(j_X+1))/2)/(h*(j_X+1));
	    posmax = q_end_X-j_X*_acclimits(0)*h; 
	}

	return std::min(dq+h*_acclimits(1), std::min(_vellimits(1), posmax));
    } else if (x<0) { //Needs a negative velocity
	
	x = -x;
	double j_x = round(sqrt(1+8*x/(h*h*_acclimits(1)))/2-1);
	double q_end_x = (-x+h*h*_acclimits(1)*(j_x*(j_x+1))/2)/(h*(j_x+1));
	double q_min_x = q_end_x-j_x*_acclimits(1)*h;      
	double X = x+h*q_min_x;
	double posmin = 0;
	if (X>0) {
	    double j_X = round(sqrt(1+8*X/(h*h*_acclimits(1)))/2-1);
	    double q_end_X = (-X+h*h*_acclimits(1)*(j_X*(j_X+1))/2)/(h*(j_X+1));
	    posmin = q_end_X-j_X*_acclimits(1)*h;
	}
	return std::max(dq+h*_acclimits(0), std::max(_vellimits(0), posmin));
    }
    else
	return 0;
         
}
