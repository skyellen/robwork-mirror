#ifndef NEWTONEULERDYNAMICS_HPP_
#define NEWTONEULERDYNAMICS_HPP_

#include <map>
#include <core/models/Q.hpp>
#include <core/models/SerialDevice.hpp>
#include <core/kinematics/Frame.hpp>
#include <core/kinematics/State.hpp>
#include <core/math/Vector3D.hpp>
#include "RigidBody.hpp"


/**
 * @brief this class calculates the newton euler dynamics of a serial
 * robot with external forces on individual joints.
 * 
 */

class NewtonEulerDynamics{
	
public :
	NewtonEulerDynamics(const rw::core::models::SerialDevice &robot,bool print=false);
	~NewtonEulerDynamics(){};
	
	void execute(rw::core::kinematics::State &state,
				 const rw::core::models::Q &q, 
				 const rw::core::models::Q &qd, 
				 const rw::core::models::Q &qdd,
				 const rw::core::math::Vector3D<double >& w0,
				 const rw::core::math::Vector3D<double >& wd0,
				 const rw::core::math::Vector3D<double >& vd0,
				 const rw::core::math::Vector3D<double >& f_end,
				 const rw::core::math::Vector3D<double >& n_end,
				 bool print=false, bool printres=false);
	
	/**
	 * @brief sets the gravity
	 */
	void setGravity(double gravity);
	
	const std::vector<double >* readTau();
	
	void printout();
	void printin();
	
	void printT(const rw::core::math::Transform3D<> &t, double b);
	
private:
	int links;
	rw::core::math::Vector3D<double > temp, pci;
	boost::numeric::ublas::bounded_matrix<double,3,3> cI;
	RigidBody* cur_body;
	double m;
	rw::core::math::Transform3D<double> Ti;

	std::vector<RigidBody* > bodies;
	
	const rw::core::models::SerialDevice *robot;
	const rw::core::kinematics::Frame* base;
	
	rw::core::math::Vector3D<double > Z;
	rw::core::math::Rotation3D<double> R;	
	
	std::vector<rw::core::math::Vector3D<double > > w;
	std::vector<rw::core::math::Vector3D<double > > wd;
	std::vector<rw::core::math::Vector3D<double > > vd;
	std::vector<rw::core::math::Vector3D<double > > vdC;
	std::vector<rw::core::math::Vector3D<double > > F;
	std::vector<rw::core::math::Vector3D<double > > Nout;

	std::vector<rw::core::math::Vector3D<double > > f;
	std::vector<rw::core::math::Vector3D<double > > n;
	std::vector<double > tau;
};

#endif /*NEWTONEULERDYNAMICS_HPP_*/
