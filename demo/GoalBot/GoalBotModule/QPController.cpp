#include "QPController.hpp"
#include "QPSolver.hpp"
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "PA10.h"


using namespace boost::numeric::ublas;
using namespace rw::math;
using namespace rw::vworkcell;
using namespace PA10;
namespace QPNamespace {
    /*  vector<double> q_min(7);
	vector<double> q_max(7);
	
	vector<double> v_min(7);
	vector<double> v_max(7);
	
	vector<double> a_min(7);
	vector<double> a_max(7);*/
    
    
    int joint_count = 7;
    double h;
    
    //  void calculateVelocityLimits(Vector& lower, Vector& upper);
    //Vector inequalitySolvePATH(const Matrix& A, const Vector& b, const Vector& lower, const Vector& upper);
    
    
    /* Used to remember which limits we have meet */
    char* lowerLimitType;
    char* upperLimitType;
    char* statusArray;
    int* accLimit;
    int* velLimit;
    int* posLimit;

    /* Variables given to the QPSolver to specify the optimization problem   */
    
    /*    int* index1;
    int* index2;
    double* lower_values;
    double* upper_values;
    double* values;
    double* bValues;
    double* res;*/
    /* Variables given to the QPSolver to specify the linear constraints */
    /*    int* cIndex1;
    int* cIndex2;
    double* cValues;
    double* ccValues;
    int* cType;
    double* lambda; */
    
    
    
    void calculateVelocityLimits(vector<double>& lower, vector<double>& upper, const vector<double>& q, const vector<double>& dq) {
	vector<double> joint_pos = q;
	vector<double> joint_vel = dq;
	double accmin, accmax, velmin, velmax, posmin, posmax;
	double x;
	for (int i = 0; i<joint_count; i++) {
	    //For the acceleration
	    accmin = h*amin[i]+joint_vel(i);
	    accmax = h*amax[i]+joint_vel(i);
	    //For the velocity
	    velmin = vmin[i];
	    velmax = vmax[i];
	    
	    //For the position
	    //If v_current<=v_max(X)
	    
	    x = qmax[i]-joint_pos(i);
	    if (x<=0) {
		posmax = 0;
		//	std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
	    } else {
		//For qmax
		double j_x = round(sqrt(1-8*x/(h*h*amin[i]))/2-1);
		double q_end_x = (x+h*h*amin[i]*(j_x*(j_x+1))/2)/(h*(j_x+1));
		double q_max_x = q_end_x-j_x*amin[i]*h;
		double X = x-h*q_max_x;
		if (X<=0){
		    posmax = 0;
		    //	    std::cout<<"Warning: Set upper pos limit to 0"<<x<<"<=0"<<std::endl;
		} else {
		    double j_X = round(sqrt(1.-8*X/(h*h*amin[i]))/2.-1);
		    double q_end_X = (X+h*h*amin[i]*(j_X*(j_X+1))/2)/(h*(j_X+1));
		    posmax = q_end_X-j_X*amin[i]*h; 
		}
	    }
	    x = joint_pos(i)-qmin[i];
	    if (x<=0)    { 
		//	std::cout<<"Warning: Set lower pos limit to 0 because"<<x<<"<=0"<<std::endl;
		posmin = 0;
	    }else {//For qmin      
		double j_x = round(sqrt(1+8*x/(h*h*amax[i]))/2-1);
		double q_end_x = (-x+h*h*amax[i]*(j_x*(j_x+1))/2)/(h*(j_x+1));
		double q_min_x = q_end_x-j_x*amax[i]*h;      
		double X = x+h*q_min_x;
		if (X<=0) {
		    posmin = 0;
		    //   std::cout<<"Warning: Set lower pos limit to 0"<<x<<"<=0"<<std::endl;
		}else {
		    double j_X = round(sqrt(1+8*X/(h*h*amax[i]))/2-1);
		    double q_end_X = (-X+h*h*amax[i]*(j_X*(j_X+1))/2)/(h*(j_X+1));
		    posmin = q_end_X-j_X*amax[i]*h;
		}
	    }
	    upper(i) = std::min(accmax,std::min(velmax, posmax));
	    lower(i) = std::max(accmin,std::max(velmin, posmin));
	    
	    if (upper(i) == accmax)
		upperLimitType[i] = 'a';
	    else if (upper(i) == velmax)
		upperLimitType[i] = 'v';
	    else
		upperLimitType[i] = 'q';
	    
	    if (lower(i) == accmin)
		lowerLimitType[i] = 'a';
	    else if (lower(i) == velmin)
		lowerLimitType[i] = 'v';
	    else
		lowerLimitType[i] = 'q';
	    
	    //Because of numerical uncertainties we need to test whether upper>lower. 
	    if (upper(i) < lower(i)) {
		lower(i) = upper(i);
		//	std::cout<<"Warning: Upper set to be lower "<<i<<std::endl;
	    }
	    //if (upper(i) == lower(i))
		//std::cout<<"Warning: Upper and Lower is equal "<<std::endl;
	}
    }
    

    vector<double> inequalitySolve(const matrix<double>& G, const vector<double>& b, const vector<double>& lower, const vector<double>& upper) {
	matrix<double> cmat = zero_matrix<double>(2*lower.size(), lower.size());
	vector<double> limits(2*lower.size());
	

	for (size_t i = 0; i<lower.size(); i++) {
	    cmat(2*i,i) = 1;
	    cmat(2*i+1,i) = -1;
	    limits(2*i) = lower(i);
	    limits(2*i+1) = -upper(i);
	}
	//std::cout<<"Calls QPSolver "<<std::endl;
	vector<double> res = QPSolver::InequalitySolve(G, -1*b, cmat, limits);
	for (size_t i = 0; i<res.size(); i++) {
	    if (res(i) > upper(i)) {
		//	std::cout<<"Warning: Invalid result for Joint "<<i<<" error "<<upper(i)-res(i)<<std::endl;
		res(i) = upper(i);
	    } else if (res(i) < lower(i)) {
		//std::cout<<"Warning: Invalid result for Joint "<<i<<" error "<<lower(i)-res(i)<<std::endl;
		res(i) = lower(i);
	    }		
	}
	return res;
    }
  
}; //End for namespace QPNamespace

using namespace QPNamespace;
using namespace PA10;

QPController::QPController(double harg) {
  h = harg;

  /* for (int i = 0; i<7; i++) {
    q_min(i) = qmin[i] ;
    q_max(i) = qmax[i];
    
    v_min(i) = vmin[i];
    v_max(i) = vmax[i];

    a_min(i) = amin[i];
    a_max(i) = amax[i];
    }*/


  //  lower_values = new double[joint_count];
  //upper_values = new double[joint_count];
  //res = new double[joint_count];

  statusArray = new char[256];
  statusArray[0] = 0;
  lowerLimitType = new char[joint_count];
  upperLimitType = new char[joint_count];

  accLimit = new int[joint_count];
  velLimit = new int[joint_count];
  posLimit = new int[joint_count];


  //int jc2 = joint_count*joint_count;
  //index1 = new int[jc2];
  //index2 = new int[jc2];
  //values = new double[jc2];
  //bValues = new double[joint_count];


  //cIndex1 = new int[jc2];
  //cIndex2 = new int[jc2];
  //cValues = new double[jc2];
  //ccValues = new double[joint_count];
  //cType = new int[joint_count];
  //lambda = new double[joint_count];
  //for (int i = 0; i<joint_count; i++) {
  //  cValues[i] = 0;
  //  cType[i] = QP_GE;
  //}


}

QPController::~QPController() {

    //  delete[] lower_values;
    //  delete[] upper_values;
    //delete[] res;
  delete[] statusArray;
  delete[] lowerLimitType;
  delete[] upperLimitType;
  delete[] accLimit;
  delete[] velLimit;
  delete[] posLimit;

  //  delete[] index1;
  //  delete[] index2;
  //  delete[] values;
  //  delete[] bValues;
  //  delete[] cIndex1;
  //  delete[] cIndex2;
  //  delete[] cValues;
  //  delete[] ccValues;
  //  delete[] cType;
  //  delete[] lambda;
}


vector<double> QPController::simpleSolve(const VelocityScrew6D<>& tcp_screw, const Device::Q& q, Device* device) {
    vector<double> tcp_vel(6);
    Vector3D<> linvel = tcp_screw.linear();
    EAA<> angvel = tcp_screw.angular();
    /*    Rotation3D<> rot;
    tcp_screw.angular().toRotation3D(rot);
    RPY<> angvel = RPY<>(rot);
    std::cout<<"rpy = "<<angvel<<std::endl;*/
    for (int i = 0; i<3; i++) {
	tcp_vel(i) = linvel(i);
	//tcp_vel(i+3) = angvel(i);
	tcp_vel(i+3) = angvel.angle()*angvel.axis()(i);
    }
    

    //  matrix<double> jac(6,7);
    //PA10::jacobian(q, jac);
    matrix<double> jac = device->bJe();    
    matrix<double> jac_inv = prod(trans(jac),PseudoInverse(prod(jac, trans(jac))));
    vector<double> sol = prod(jac_inv, tcp_vel);
    
    //    std::cout<<"SimpleSol Desired = "<<tcp_vel<<std::endl;
    //std::cout<<"SimpleSol Actual = "<<prod(jac,sol)<<std::endl;
    return sol;
}

vector<double> QPController::solve(const VelocityScrew6D<>& tcp_screw, const Device::Q& q, const Device::Q& dq, Device* device) {

    vector<double> tcp_vel(6);
    Vector3D<> linvel = tcp_screw.linear();
    EAA<> angvel = tcp_screw.angular();
    /*    Rotation3D<> rot;
    tcp_screw.angular().toRotation3D(rot);
    RPY<> angvel = RPY<>(rot);
    std::cout<<"rpy = "<<angvel<<std::endl;*/
    for (int i = 0; i<3; i++) {
	tcp_vel(i) = linvel(i);
	//tcp_vel(i+3) = angvel(i);
	tcp_vel(i+3) = angvel.angle()*angvel.axis()(i);
    }


    //  tcp_vel *= .1;
    
    //  matrix<double> jac(6,7);
    //  PA10::jacobian(q, jac);
    matrix<double> jac = device->bJe();
    matrix<double> A( prod(trans(jac),jac) );
    vector<double> b( prod(trans(jac),tcp_vel) );
    
   
    vector<double> lower(joint_count);
    vector<double> upper(joint_count);
    calculateVelocityLimits(lower, upper, q, dq);

    vector<double> sol1 = inequalitySolve(A, b, lower, upper);
    //    std::cout<<"QPRes Desired = "<<tcp_vel<<std::endl;
    //std::cout<<"QPRes Actual = "<<prod(jac,sol1)<<std::endl;
    return sol1;
    
}

