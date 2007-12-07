#include "XQPController.hpp"

#include <rwlibs/algorithms/qpcontroller/QPSolver.hpp>
#include <rw/math/Math.hpp>
#include <float.h>

using namespace rwlibs::algorithms;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::algorithms::qpcontroller;

using namespace boost::numeric::ublas;


XQPController::XQPController(DeviceModel* device, 
							 Frame* controlFrame,
							 State& state, 
							 double dt):
	_device(device),
	_controlFrame(controlFrame),
	_state(state),
	_dt(dt),
	_dof(device->getDOF())
{
    _qlower = _device->getBounds().first;
    _qupper = _device->getBounds().second;

    _dqlimit = _device->getVelocityLimits();
	
    _ddqlimit = _device->getAccelerationLimits();
	
}

XQPController::~XQPController()
{
}


Q XQPController::inequalitySolve(const matrix<double>& G, 
 				      			 const vector<double>& b, 
 				      			 const vector<double>& lower, 
 				      			 const vector<double>& upper, 
 				      			 const std::list<Constraint>& constraints) {
									 
    matrix<double> cmat = zero_matrix<double>(2*lower.size() + constraints.size(), lower.size());
    vector<double> limits(2*lower.size() + constraints.size());
    for (size_t i = 0; i<lower.size(); i++) {
        cmat(2*i,i) = 1;
        cmat(2*i+1,i) = -1;
        limits(2*i) = lower(i);
        limits(2*i+1) = -upper(i);        
    }
    
    //TODO Add the constraints
    size_t index = lower.size(); 
    for (std::list<Constraint>::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
    	const Constraint& c = *it;
    	vector<double> row = prod(trans(c._jac.m()), c._direction.m());
    	for (size_t i = 0; i<row.size(); i++) {
    		cmat(index, i) = row(i);
    	}
    	limits(index) = c._velocity;
    }
    vector<double> qstart = (lower+upper)/2.0;
    QPSolver::Status status;
    vector<double> res = QPSolver::InequalitySolve(G, -1*b, cmat, limits, qstart, status);

    /*    for (size_t i = 0; i<res.size(); i++) {
        if (res(i) > upper(i)) {
            res(i) = upper(i);
        } else if (res(i) < lower(i)) {
            res(i) = lower(i);
        }               
        }*/

    return Q(res);
}

/*namespace  {
    double max3(double a, double b, double c) {
        return std::max(a, std::max(b,c));
    }
    
    double min3(double a, double b, double c) {
        return std::min(a, std::min(b,c));
    }
}*/

/*void XQPController::calculateVelocityLimits(vector<double>& lower,
 							 				vector<double>& upper,
 							 				const Q& q,
 							 				const Q& dq) {
 							 				*/
   /* 
    for (size_t i = 0; i<_dof; i++) {       
        lower(i) = std::max(-fabs(_dqlimit(i)), 
                            -fabs(q(i)-_qlower(i))/_dt);
            
        upper(i) = std::min(fabs(_dqlimit(i)),	
                            fabs(_qupper(i)-q(i))/_dt);      
    }
    */
	/*for (size_t i = 0; i<_dof; i++) {			    
		lower(i) = max3(-fabs(_dqlimit(i)), 
		                -fabs(q(i)-_qlower(i))/_dt,
		                dq(i) - _ddqlimit(i)*_dt);
		
		upper(i) = min3(fabs(_dqlimit(i)), 
		                fabs(_qupper(i)-q(i))/_dt,
		                dq(i) + _ddqlimit(i)*_dt);		
	}
	
}*/

void XQPController::calculateVelocityLimits(vector<double>& lower,
                                           vector<double>& upper,
                                           const Q& q,
                                           const Q& dq)
{
    Q joint_pos = q;
    Q joint_vel = dq;

    double accmin, accmax, velmin, velmax, posmin, posmax;
    double x;

	const Q& _qmax = _qupper;
	const Q& _qmin = _qlower;

	const Q& _vmax = _dqlimit;
	const Q& _vmin = -_dqlimit;

	const Q& _amax = _ddqlimit;
	const Q& _amin = -_ddqlimit;
    for (size_t i = 0; i<_dof; i++) {
        //For the acceleration
        accmin = _dt*_amin[i]+joint_vel(i);
        accmax = _dt*_amax[i]+joint_vel(i);
        //For the velocity
        velmin = _vmin[i];
        velmax = _vmax[i];
        //For the position
        //If v_current<=v_max(X)
        x = _qmax[i]-joint_pos(i);
        if (x<=0) {
            posmax = 0;
            //  std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
        } else {
            //For qmax            
			double j_x = Math::Round(sqrt(1-8*x/(_dt*_dt*_amin[i]))/2-1);
            double q_end_x = (x+_dt*_dt*_amin[i]*(j_x*(j_x+1))/2)/(_dt*(j_x+1));
            double q_max_x = q_end_x-j_x*_amin[i]*_dt;
            double X = x-_dt*q_max_x;
            if (X<=0){
                posmax = 0;
                //          std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
            } else {
                double j_X = Math::Round(sqrt(1.-8*X/(_dt*_dt*_amin[i]))/2.-1);
                double q_end_X = (X+_dt*_dt*_amin[i]*(j_X*(j_X+1))/2)/(_dt*(j_X+1));
                posmax = q_end_X-j_X*_amin[i]*_dt; 
            }
        }
        x = joint_pos(i)-_qmin[i];
        if (x<=0)    { 
            //  std::cout<<"Warning: Set lower pos limit to 0 because"<<x<<"<=0"<<std::endl;
            posmin = 0;
        }else {//For qmin      
            double j_x = Math::Round(sqrt(1+8*x/(_dt*_dt*_amax[i]))/2-1);
            double q_end_x = (-x+_dt*_dt*_amax[i]*(j_x*(j_x+1))/2)/(_dt*(j_x+1));
            double q_min_x = q_end_x-j_x*_amax[i]*_dt;      
            double X = x+_dt*q_min_x;
            if (X<=0) {
                posmin = 0;
                //   std::cout<<"Warning: Set lower pos limit to 0"<<x<<"<=0"<<std::endl;
            }else {
                double j_X = Math::Round(sqrt(1+8*X/(_dt*_dt*_amax[i]))/2-1);
                double q_end_X = (-X+_dt*_dt*_amax[i]*(j_X*(j_X+1))/2)/(_dt*(j_X+1));
                    posmin = q_end_X-j_X*_amax[i]*_dt;
            }
        }
        upper(i) = std::min(accmax,std::min(velmax, posmax));
        lower(i) = std::max(accmin,std::max(velmin, posmin));
        
           
        //Because of numerical uncertainties we need to test whether upper>lower. 
        if (upper(i) < lower(i)) {
            lower(i) = upper(i);
        }
    }
}
 

Q XQPController::solve(const rw::math::Q& q,		
                       const rw::math::Q& dq,
					   const rw::math::VelocityScrew6D<>& tcpscrew, 
					   const std::list<Constraint>& constraints) {
	
    if (q.size() != _dof) 
        RW_THROW("Length of input configuration does not match the DeviceModel");

    vector<double> tcp_vel(6);
    Vector3D<> linvel = tcpscrew.linear();
    EAA<> angvel = tcpscrew.angular();

    tcp_vel(0) = linvel(0);      
    tcp_vel(1) = linvel(1);
    tcp_vel(2) = linvel(2);
    tcp_vel(3) = angvel.angle()*angvel.axis()(0);
    tcp_vel(4) = angvel.angle()*angvel.axis()(1);
    tcp_vel(5) = angvel.angle()*angvel.axis()(2);
    
    _device->setQ(q, _state);
	
    matrix<double> jac = _device->baseJframe(_controlFrame,_state).m();
	
    //trim jacobian to remove world z-rotation
    //    matrix_range<matrix<double> > jac(jac6, range(0,6), range(0, jac6.size2()));

    matrix<double> A = prod(trans(jac),jac);
    vector<double> b = prod(trans(jac),tcp_vel);
   
    vector<double> lower(_dof);
    vector<double> upper(_dof);
	
    calculateVelocityLimits(lower, upper, q, dq);
    Q sol1 = inequalitySolve(A, b, lower, upper, std::list<Constraint>());

    return sol1;

}
