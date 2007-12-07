#ifndef RWLIBS_ALGORITHMS_XQPCONTROLLER_HPP_
#define RWLIBS_ALGORITHMS_XQPCONTROLLER_HPP_

#include <list>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/DeviceModel.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/numeric/ublas/matrix.hpp>

namespace rwlibs {
namespace algorithms {


class Constraint {
public:
	rw::math::Jacobian _jac;
	rw::math::Q _direction;
	double _velocity;
public:
	Constraint(rw::math::Jacobian& jac, 
			   rw::math::Q& direction, 
			   double velocity):
		_jac(jac),
		_direction(direction),
		_velocity(velocity)
   {
   }

	
};

/**
 * @brief An extended version of the QPController
 * 
 * Notice: Still under development
 * 
 * The XQPController extends the QPController by providing the user
 * with the possibility of adding constraints on the motion. This method
 * follows the algorithm described in [1].
 * 
 * [1]: Write name of paper when published
 */
class XQPController
{
public:
    /**
     * @brief Constructs XQPController
     * @param device [in] Device to control
     * @param controlFrame [in] Frame to control
     * @param state [in] State specifying how frames are assempled
     * @param dt [in] time step size
     */
	XQPController(rw::models::DeviceModel* device, 
				  rw::kinematics::Frame* controlFrame,
				  rw::kinematics::State& state, 
				  double dt);

	/**
	 * @brief Destructor
	 */
	virtual ~XQPController();
	
	/**
	 * @brief Solves the inverse kinematics problem
	 * 
	 * Notice that if the constraints cannot be fullfilled, no quarantees for the
	 * return value is given.
	 * 
	 * @param q [in] Current configuration of the device
	 * @param dq [in] Current joint velocities for the device
	 * @param tcpvel [in] Desired tool velocity seen in base frame
	 * @param constraints [in] List of constraints
	 */ 
	rw::math::Q solve(const rw::math::Q& q,				
	                  const rw::math::Q& dq,
			          const rw::math::VelocityScrew6D<>& tcpvel, 
			          const std::list<Constraint>& constraints);
	
private:
	rw::models::DeviceModel* _device;
	rw::kinematics::Frame* _controlFrame;
	rw::kinematics::State _state;
	double _dt;
	size_t _dof;
	
	rw::math::Q _qlower;
	rw::math::Q _qupper;
	rw::math::Q _dqlimit;
	rw::math::Q _ddqlimit;
	
	/**
	 * Solves the inequality problem 1/2 x^T.G.x+b^T.x subject to lower <= x <= upper
	 * and the constraints in the constraint list 
	 */
    rw::math::Q inequalitySolve(const boost::numeric::ublas::matrix<double>& G, 
     				            const boost::numeric::ublas::vector<double>& b, 
     				            const boost::numeric::ublas::vector<double>& lower, 
     				            const boost::numeric::ublas::vector<double>& upper, 
     				            const std::list<Constraint>& constraints);

    /**
     * Calculates the velocity limits 
     */
    void calculateVelocityLimits(boost::numeric::ublas::vector<double>& lower,
                                 boost::numeric::ublas::vector<double>& upper,
     							 const rw::math::Q& q,
     							 const rw::math::Q& dq);

};

} //end namespace algorithms
} //end namespace rwlibs

#endif /*RWLIBS_ALGORITHMS_XQPCONTROLLER_HPP_*/
