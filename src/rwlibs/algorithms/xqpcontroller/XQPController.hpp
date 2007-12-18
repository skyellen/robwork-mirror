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
     * @brief Constraint for the XQPController   
     * 
     * A constraint is constructed as \f$J^Td \geq v \f$ where
     * \f$J\f$ is the jacobian of the frame to be controlled, \f$d\f$ the direction 
     * of the constraint and \f$v\f$ the maximal velocity.
     */
    class Constraint {
    public:
        rw::math::Jacobian _jac;
        rw::math::Q _direction;
        double _velocity;
    public:
        /**
         * @brief Constructor for constraint
         * 
         * A constraint is constructed as \f$J^Td \geq v \f$ where
         * \f$J\f$ is the jacobian of the frame to be controlled, \f$d\f$ the direction 
         * of the constraint and \f$v\f$ the maximal velocity.
         * 
         * @param J [in] Jacobian of the frame associated with the consraint
         * @param d [in] direction of the constraint
         * @param v [in] the velocity of the constraint
         */
        Constraint(rw::math::Jacobian& J, 
                   rw::math::Q& d, 
                   double v):
            _jac(J),
            _direction(d),
            _velocity(v)
       {
       }
    };
    
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
	
	/**
	 * @brief Enumeration used to specify frame associated with the projection
	 */
	enum ProjectionFrame { BaseFrame = 0, /** Robot Base Frame */ 
	                   ControlFrame  /**The Frame specified as the controlFrame*/
	                  };
	
	/**
	 * @brief Setup the projection
	 * 
	 * The traditional relationship between device Jacobian, joint velocities and tool velocity
	 * is given by \f$J\dot{q}=\dot{q}\f$. To ignore certain degrees of freedom or put more
	 * emphasis (with respect to the least square solution) on some we can multiply with 
	 * \f$P\f$ to get \f$P J\dot{q}=P \dot{x}\f$. 
	 * 
	 * \f$P\f$ needs to have exactly 6 columns, however the number of row may be less than 6.
	 * Use the \b space flag to specify in which space the projection should occur. 
	 * 
	 * Usage: Setup to ignore tool roll
	 * \code
	 * XQPController* xqp = new XQPController(device, device->getEnd(), state, dt)
	 * boost::numeric::ublas::matrix<double> P = boost::numeric::ublas::zero_matrix<double>(5,6);
	 * for (int i = 0; i<5; i++)
	 *     P(i,i) = 1;
	 * xqp->setProjection(P, XQPController::ControlFrame);
	 * \endcode
	 * 
	 * Usage: Increase the weight of the z-coordinate relative to the base
	 * \code
	 * XQPController* xqp = new XQPController(device, device->getEnd(), state, dt);
	 * boost::numeric::ublas::matrix<double> P = boost::numeric::ublas::identity_matrix<double>(6);
	 * P(2,2) = 100; //Increase the least square weight with a factor of 100 
	 * xqp->setProjection(P, XQPController::BaseFrame);
	 * \endcode
	 * 
	 * @param P [in] The projection matrix
	 * @param space [in] The space in which to apply the projection
	 */
	void setProjection(const boost::numeric::ublas::matrix<double>& P, ProjectionFrame space);
	
private:
	rw::models::DeviceModel* _device;
	rw::kinematics::Frame* _controlFrame;
	rw::kinematics::State _state;
	double _dt;
	size_t _dof;
	
	ProjectionFrame _space;
	boost::numeric::ublas::matrix<double> _P;
	
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
